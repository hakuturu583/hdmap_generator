# 市街地以外の環境 — 露天掘り鉱山の IR と生成 — 設計

*2026-10 設計・実装。*

## 0. 実装状況

| 部品 | 場所 |
| --- | --- |
| 地形（TIN）: `Tin` / `TinSurface` / `TinBreakline` / `TinLocator` | `crates/roadgen-core/src/terrain.rs` |
| 土地被覆: `LandCover` | `crates/roadgen-core/src/land_cover.rs` |
| 複合地物: `Site` | `crates/roadgen-core/src/site.rs` |
| 路面: `Pavement` / `PavedSurface` / `UnpavedSurface` / `PavementProfile`、`Road.pavement` | `crates/roadgen-core/src/semantics.rs`、`map.rs` |
| 土手: `MapObjectKind::Barrier { kind, height }` | `crates/roadgen-core/src/semantics.rs` |
| 検証（TIN の折り返し、折れ線が TIN の辺に乗っているか、参照の双方向性） | `crates/roadgen-core/src/validation.rs` |
| IR ダンプ（新しい地物は空なら出力しない＝既存の地図の fingerprint は不変） | `crates/roadgen-trace/src/ir.rs` |
| 環境の切り替え: `Environment::{Urban, Mine}` / `lay_out` / `generate` | `crates/roadgen-carla/src/environment/mod.rs` |
| 共通: 地形（TIN の組み立て、全道路の縁を折れ線に、起伏、基準高さ）、土地被覆の登録 | `crates/roadgen-carla/src/environment/{land,tin,noise}.rs` |
| 共通: 建物（CGA 文法、道路沿いの区画、`Placement` で道路・範囲を絞る） | `crates/roadgen-buildings`（`generate_with`） |
| 共通: 平面幾何（内外判定・距離・線分インデックス） | `crates/roadgen-core/src/geometry/plan.rs` |
| 鉱山固有: ピット・ランプ・盛土・ヤードの形、土手、Site | `crates/roadgen-carla/src/environment/mine/` |
| Python: `Map.generate_land()`（街の地形）、`generate_mine(yard_buildings=...)` | `crates/roadgen-python` |
| CARLA: 地形を TIN から構築、砂利・岩の材質、土手のメッシュ | `crates/roadgen-carla/src/{ground,terrain,barriers}.rs` |
| OpenDRIVE: `<lane><material>`（書き出しと読み戻し）、`<object type="barrier">` + `<repeat>` | `crates/roadgen-opendrive` |
| OpenStreetMap: `surface=*`、`landuse=quarry` / `man_made=spoil_heap` の面、`barrier` の way | `crates/roadgen-osm` |
| Lanelet2: 土手を `road_border` の linestring として出力 | `crates/roadgen-lanelet2` |
| Python: `Map.generate_mine()`、`site` / `tin_mesh` / `tin_breaklines` / `terrain_height` / `land_cover` / `road_pavement`、`add_road(pavement=...)` | `crates/roadgen-python` |

## 1. 何が足りなかったか

CARLA の出力は「道路網から、その見た目を作る」設計で、地面は道路の高さを外側へ平らに広げたものだった（`ground::Field`）。市街地ではそれで正しい。道路網は土地の起伏について何も言わないので、言われていない起伏を作らない、という判断である。

露天掘り鉱山では地形そのものが環境の本体である（ベンチ、ランプ、ズリ山、貯鉱山）。そこで、

- IR が **地形を「述べられるもの」として持てる** ようにし、
- それを埋める **生成処理** を、CARLA アセット生成の **環境の分岐** として `roadgen-carla` の中に足した（`Environment::Urban` が市街地＝建物、`Environment::Mine` が鉱山）。環境ごとにクレートを分けない。

生成処理は IR だけを書き、パッケージは環境によらず同じ出力処理で書く。生成結果は IR に入るので、CARLA 以外の形式にも届く。

## 2. IR の設計 — GDF 5.1 の考え方に寄せる

IR は GDF 5.1 から「考え方だけ」を借りている（README の Design lineage）。今回もその方針どおり、GDF の機能カタログ・コード・XML は採らず、テーマの切り方と原則だけを借りた。

| GDF | IR |
| --- | --- |
| Level 0（幾何） | `Point3` / `Polyline3` / `Tin` |
| Level 1（単純地物） | `TinSurface`、`TinBreakline`、`LandCover`（＋既存の `Road`、`MapObject`） |
| Level 2（複合地物） | `Site` |
| 属性 | `Road.pavement`（Pavement Status → Surface Type の 2 段） |
| 関係 | `LandCover.draped_on`、`TinBreakline.surface`、`site` の相互参照 |

### 2.1 Terrain Elevation テーマ → `terrain.rs`

- `TinSurface` は地物（ID: `tin/<site>`）。
- 頂点と三角形（`Tin`）は **その幾何であって地物ではない**。三角形に入力由来の安定した ID は付けられず、付けると再生成した地図どうしの比較が無意味になるため。
- `TinBreakline` は地物（ID: `breakline/<site>/bench/3/crest` など）。ベンチの肩・法尻、ズリ山の肩・法尻、道路の縁（`road_edge`）。
- **不変条件**: 折れ線の各頂点は TIN の節点であり、各線分は TIN の辺である。`validate()` が検査する。
- 道路は地形に含めない。地形は道路の下の地面で、生成器は道路の縁を折れ線として TIN に入れるので、両者は頂点単位で一致する。

### 2.2 Land Cover And Use テーマ → `land_cover.rs`

形（地形）と意味（土地被覆）を分ける。`LandCover { kind, area, draped_on }`。`kind` は `Building.kind` と同じく自由文字列（`pit` / `waste_dump` / `stockpile` / `pad`）。GDF のクラスコードを採らない方針に合わせた。

### 2.3 Road Element の属性 → `Pavement`

`Pavement::Paved(PavedSurface) | Pavement::Unpaved(UnpavedSurface)` の 2 段。「未舗装なのにアスファルト」は型で表現できない。摩擦係数・粗さは IR に入れず、OpenDRIVE の出力処理が `Pavement` から既定値を引く（`roadgen-opendrive/src/materials.rs`）。

### 2.4 Level 2 → `Site`

`Site { kind, land_covers, surfaces, roads }`。構成要素側も `site` を持ち、双方向に参照する（`Road`↔`Lane`、`Building`↔`BuildingPart` と同じ）。

### 2.5 土手 → `MapObjectKind::Barrier`

GDF でどのテーマに属するか（Road Furniture か Structures か）は本文で確認できていない。既存の道路付属物モデル（種類・幾何・関係する車線）に収まるので、ここに置いた。

## 3. 生成 — `roadgen_carla::environment::mine`

```
Environment::Mine(MineSpec + seed)
   │
environment::lay_out(&mut MapBuilder)      ランプ・運搬路・交差点（道路は builder に追加）
   │
MapBuilder::finish()
   │
environment::generate(&mut UnvalidatedMap) 地形 TIN と折れ線・土地被覆・土手・Site
   │
validate()
```

2 段階なのは、地形を **生成済みの道路の縁** に合わせて作るため。

1. **ピット**: 床は楕円。各ベンチの法尻・肩は床の外形を法線方向にオフセットした平行曲線（凸なので折れない）。法面は `face_angle`、段差は `bench_height`、ベンチ幅は `bench_width`。
2. **ランプ**: リム上の `ramp.start` から一定勾配 `ramp.grade` で壁を螺旋状に下る。高さ z における壁の平均位置に **ランプの外側の縁** を合わせるので、外側は切土、内側は盛土になる。リムからの入りは 40 m かけてなめらかに曲げる。入口はリムから外へ離した 2 次ベジェで、その先に交差点。
3. **運搬路**: 交差点からズリ山・貯鉱山の法尻手前まで。3 本の腕を交差点で相互に接続（接続道路は builder が生成）。車線は 走行 2 + 路肩 2、白線なし、`Unpaved(Gravel)`。
4. **ズリ山・貯鉱山**: 安息角（既定 37°）の円錐台。法尻と肩を折れ線にする。
5. **地形**: spade の制約付き三角形分割。自由点 = 周辺の格子（`ground.cell`）、ピットの床、盛土の天端。制約線 = ベンチ・盛土の肩と法尻、道路の縁。ランプが通る所のベンチ線は道路の縁から 3 m 以内を切り取る。辺にならなかった線分は折れ線から外す（§2.1 の不変条件を生成側で保証）。周辺は seed 付きのフラクタル雑音で緩やかに起伏させ、施設の周り（20 m 以内）は平らに保つ。
6. **土手**: ランプのピット側の路肩に `Barrier { kind: "berm", height: berm_height }`。
7. **Site** で全体を束ねる。

同じ `MineSpec`（seed を含む）からは常に同じ鉱山ができる。ID は名前と地物の種類から決まる。

## 3.5 市街地と鉱山で共有しているもの

環境ごとに違うのは「形」だけで、組み立て方は共有する。

```
                    共通の段                          市街地                     鉱山
道路の配置          MapBuilder                         利用者が描く               ランプ・運搬路・交差点
建物                roadgen_buildings::generate_with   Site に属さない全道路沿い  運搬路沿い・ヤード内のみ
                    （区画割り＋CGA 文法）             （town などのプリセット）  （industrial プリセット）
地形                environment::land::generate        Base::Roads、形なし        Base::Level(リム)、
                    （全道路の縁＝折れ線、格子、起伏、                            ベンチ・盛土の線、床・天端
                     TIN、IR へ登録）
土地被覆            land::cover                        —                          pit / waste_dump / stockpile / pad
CARLA 出力          surfaces / terrain / barriers      IR から（環境によらず同じ）
```

- 地形は「基準高さ（一定 or 道路から決まる高さ）」と「形（折れ線・平坦面・穴・平地）」を渡すだけで、市街地も鉱山も同じ関数で作る。道路から決まる高さには、CARLA 出力が地面を作るときと同じ `ground::Field` を使う。
- 建物は、鉱山のヤードにも市街地と同じ区画割り・同じ文法で建てる。区画は道路沿いなので、ヤードの建物も運搬路に面する。
- 市街地は、鉱山など他の Site に属する道路には建物を並べない。

## 4. 各出力処理

| 形式 | 地形 | 土地被覆 | 路面 | 土手 |
| --- | --- | --- | --- | --- |
| CARLA | TIN の節点と折れ線から地面を作り直す（道路の縁に 1 m 以内で接続） | 三角形ごとに材質を分ける（ピット・盛土＝岩、ヤード＝砂利、他＝草）。外形を制約線に入れ、境界を外形に揃える | 未舗装 → `M_Road_Gravel` | 台形断面のメッシュ（`Terrain`） |
| OpenDRIVE | 出力しない（道路の標高のみ） | 出力しない | `<lane><material surface friction roughness>`。読み戻しも可 | `<object type="barrier" subtype="berm">` + `<repeat>` |
| OpenStreetMap | 出力しない（`check()` で警告） | `landuse=quarry`、`man_made=spoil_heap`、`landuse=industrial` の閉じた way | `surface=*` | `barrier=yes` の way |
| Lanelet2 | 出力しない | 出力しない | 出力しない | `type=road_border` の linestring |
| SUMO / ClipGT / GPUDrive | 出力しない | 出力しない | 出力しない | 出力しない |

## 5. IR にあるものはすべてアセットに出す

環境（市街地・鉱山）によらず、CARLA アセットは IR から作る。鉱山マップに利用者が足した道路（歩道・縁石付きなど）も、他の道路と同じに描かれる。

| IR | CARLA アセット |
| --- | --- |
| 車線（全種別）・幅・横断勾配 | 路面・歩道・縁石・側溝のメッシュ |
| 車線の高さ `LaneHeight` | 測点ごとにその高さで描く（以前は歩道を設定の縁石高さで一律に上げていた。IR が平らな歩道だけ、従来どおり設定値を使う） |
| 区画線 | 塗装メッシュ（持ち上がった車線の上に載る） |
| 舗装 `Road.pavement` | 舗装の種類ごとの材質（アスファルト・コンクリート・石畳・砂利・土・砂・岩）。道路の途中で変わる所には断面を挿入し、そこで材質を切り替える |
| 信号・標識 / 停止線・横断歩道 | 小物（props）/ 塗装 |
| 土手 `Barrier` | 種類ごとの形：berm 等は安息角の斜面（砂利）、wall はコンクリートの直立壁、それ以外は鋼の直立体 |
| 建物 | 地図の FBX 内・props・出力しない（設定） |
| 地形 TIN と折れ線 | 地面（折れ線を辺として保持） |
| 土地被覆 | 地面の材質（外形を制約線に入れて境界を揃える） |

鉱山の地形 TIN は、鉱山自身の道路だけでなく **マップ上のすべての道路** の縁を折れ線として取り込む。ベンチの線や周辺の格子点は、どの道路からも離して置く。だから後から足した道路も地形と頂点単位で一致する。

## 6. 既知の制約・今後

- 1 鉱山 1 ピット、ランプは螺旋 1 本。スイッチバック、複数ピット、坑内掘りは対象外。
- ベンチ線をランプ付近で切り取った端で、細い三角形の切れ込みが出ることがある。
- CARLA の地面は TIN の範囲（施設の外接矩形＋`ground_margin`）で終わる。
- 新しい CARLA 材質の Poly Haven スラッグ（`rocky_trail`、`aerial_rocks_02`）は既定値で、実在は未確認（開発環境から API に接続できなかったため）。取得スクリプトが存在しないスラッグを報告するので、その場合はマニフェストを差し替えればよい。
- `use_carla_materials=True`（既定）だと CARLA が路面をアスファルトに塗り替える。`check()` が警告する。
