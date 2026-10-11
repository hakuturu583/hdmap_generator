# 高速道路環境 — トンネル・橋・高架の IR と生成 — 設計（レビュー用）

*2026-10 設計。実装前のレビュー用。§11 にレビューで決めてほしい点をまとめた。*

## 0. 目的と範囲

鉱山環境（[`mine-design.md`](mine-design.md)）と同じ考え方で、**トンネル・橋・高架を含む高速道路** を IR に表現し、生成できるようにする。高速道路が市街地とつながる場合（インターチェンジ、都市高速の高架、高速道路の終点から一般道への移行）の生成規則も決める。

鉱山から引き継ぐ原則は次の 4 つ。

1. **環境ごとの生成関数は作らない。** 「高速道路」という生成関数も持たない。利用者が置く **部品**（回廊・橋・トンネル・丘・川・インターチェンジ）と道路の組み合わせで地図を作る。鉱山・市街地・高速道路を 1 つの地図に同時に置ける。
2. **道路の形は利用者が決める。** 補助関数（`Pit.ramp()` と同様の `Corridor.carriageway()`、`Corridor.interchange()` など）は点列や計画を返すだけで、利用者がそれを `add_road` に渡す。補助関数を使わず点列を手で描いてもよい。
3. **生成処理は IR だけを書く。** 出力処理は環境によらず同じで、生成結果は CARLA 以外の形式にも届く。
4. **IR は GDF 5.1 の「考え方だけ」を借りる。** 機能カタログ・コード・XML は採らない（README の Design lineage）。

範囲外：鉄道・水路の橋（`Structure` の上を通れるものは道路に限る。下を通るものとしては川を扱う）、料金所、サービスエリア・パーキングエリア（§10 で将来の拡張として触れる）。

## 1. 現状 — 何が足りないか

コードを調べた結果（`file:line` は 2026-10-11 時点の `main`）。

| 項目 | 現状 | 問題 |
| --- | --- | --- |
| IR | 道路の高さは 3D 参照線 `Curve3` の z だけで持つ（`roadgen-core/src/map.rs:203`）。トンネル・橋の概念はない | 「この区間は橋」と言えない |
| 道路の分類 | `RoadType { Town, Rural, Motorway, LowSpeed, Pedestrian }`（`semantics.rs:62`）。OpenDRIVE 由来 | ランプ（slip road）、本線、側道を区別できない |
| 検証 | 平面で交差する 2 本の道路が交差点なしで交わっても、高さの差を問わず通る（`validation.rs` に交差の検査がない）。web デモの "A town, generated" は同じ高さで交差点なしに交差している | 高速道路と街路の平面交差を検出できない |
| 縦断線形 | 区間ごとに一定勾配（`Alignment.line(length, rise)`）。縦断曲線がない | 勾配の変化点が折れになる |
| 合流・分流 | `connect_lanes` と幅のテーパー、`cross_sections` で手で組む | 加減速車線を組むのが大変 |
| OpenDRIVE | `<tunnel>` / `<bridge>` は常に空（`roadgen-opendrive/src/lib.rs:1743`）。読み込み側も無視し、注記も出さない | |
| OSM | `layer` / `bridge` / `tunnel` を書かない。`highway=motorway_link` もない | 立体交差が「交差しているのに節点を共有しない道」として検証ツールに指摘される |
| Lanelet2 / SUMO / ClipGT | z はそのまま出る | 構造物の情報はない |
| GPUDrive | 2D 形式。上の道路の `road_edge` が平面上で下の道路を横切る | 衝突判定の壁が下の道路を塞ぐ |
| CARLA 地面 | 地面の高さは「近くの道路で最も低い点」から作る（`ground.rs:66`）。土地は道路の平面外形で切り抜く（`terrain.rs:198`） | 高架は盛土の上に載り、トンネルは掘割になる。高さの違う外形が交わる所で z が 1 つに潰れ、スパイクが出る |
| 地形 TIN | 全道路の縁が折れ線になり、`Base::Roads` が全車線の中心線から高さを取る（`environment/land.rs:452`） | 地面が高架の床版の高さまで引き上げられる |
| CARLA 道路メッシュ | 片面のリボンで、裏面・側面がない（`surfaces.rs:675`） | 高架を下から見ると透けて見え、点群にも裏面が出ない |
| 建物 | 部品の外にある、Site に属さない全道路沿いに建てる | 高速道路沿い・高架の下にも建つ |

**地形 TIN は 2.5D（1 つの平面位置に 1 つの高さ）のままでよい。** 橋の下も、トンネルの上も、地面は 1 枚である。壊れているのは「すべての道路は地面に接している」という前提のほうで、構造物の区間ではその前提を外せばよい（§3.7）。

## 2. GDF からの写像

### 2.1 GDF（と派生形式）がトンネル・橋をどう表すか

GDF 5.0 / 5.1 の本文は有償で、今回は直接読めていない。以下は二次資料による。**[確]** は資料で確認したもの、**[推]** は推定。

- **[確]** 初期の GDF（3.0 の元になった CEN 案）には、橋とトンネルをまとめた地物 **Brunnel**（bridge + tunnel）があり、"Brunnel Type"（bridge / viaduct / aqueduct / tunnel）で種類を表す。関係 **"X over Y through Brunnel"** が、交差点でない場所で道路要素どうしの上下を表す。
- **[確]** GDF 5.0 / 5.1 には `ROADS_AND_FERRIES` テーマとは別に `STRUCTURES` テーマがある。
- **[確]** GDF 系の実用形式（TomTom MultiNet）は **両方** を持つ。道路要素の属性 `PARTSTRUC`（0 なし / 1 トンネルの一部 / 2 橋の一部）と、別のテーマに置かれた構造物地物である。端点の相対レベルは `F_ELEV` / `T_ELEV` で、0 が地表を表す。HERE も相対レベル（Z-Level）を持つ。
- **[確]** Form of Way は GDF の属性で、値に MOTORWAY / MULTIPLE_CARRIAGEWAY / SINGLE_CARRIAGEWAY / ROUNDABOUT / SLIPROAD などがある（OpenLR の定義による）。MOTORWAY は「自動車専用、分離された 2 つ以上の車道、平面交差なし」と定義される。
- **[確]** GDF の Level 2 では、道路要素と交差点をまとめて複合地物 "Road" と "Intersection" にする。インターチェンジは 1 つの Intersection で、ランプ（Form of Way = slip road）は Level 1 の道路要素のまま残る。
- **[推]** GDF 5.1 Part 2（自動運転向けの車線モデル）も、橋・トンネルは Part 1 の道路要素の属性として扱っている。3D の構造物を新しく定義してはいないと見られる。

### 2.2 IR への写像

| GDF の考え方 | IR（新規は **太字**） |
| --- | --- |
| Level 0（幾何） | `Curve3`、`Tin`、**構造物の幾何（床版外形・橋脚・坑口）** |
| Structures テーマの構造物（Bridge / Tunnel） | **`Structure`**（Level 1 の地物） |
| 道路要素の属性「構造物の一部」（PARTSTRUC） | **`Road.structure: StructureProfile`**（測点の範囲 → `StructureId`） |
| 関係 "X over Y through Brunnel" | **`GradeSeparation`**（関係そのものを地物として持つ） |
| 相対レベル（F_ELEV / T_ELEV、Z-Level） | **持たない**。実際の z があるので、出力処理が導出する（OSM の `layer` など） |
| Form of Way | **`Road.form: FormOfWay`** |
| Level 2 Intersection（インターチェンジ） | **`Interchange`**（Level 2 の複合地物） |
| Level 2 の場所（鉱山と同じ） | `Site`（kind `motorway`：回廊の土地被覆と道路をまとめる） |
| Terrain Elevation / Land Cover | `TinSurface` / `TinBreakline` / `LandCover`（**折れ線と被覆の種類を追加**） |

GDF と同じく、**属性と地物の両方** を持つ。道路の側は「この区間はどの構造物の上（中）か」を、構造物の側は「どの道路のどの区間を通すか」を持ち、互いに参照する。`Road`↔`Lane`、`Site`↔構成要素と同じ双方向の参照で、`validate()` が一致を検査する。相対レベルは持たない。IR は実際の 3D 座標を持っており、レベルを別に持つと 2 つの値が食い違いうるからである。

## 3. IR の設計

### 3.1 `Structure` — 構造物（Level 1）

新しいモジュール `roadgen-core/src/structure.rs` と、アリーナ `Map.structures` を追加する。

```rust
/// 道路を地面から離して通すもの：橋、高架、トンネル。
pub struct Structure {
    pub id: StructureId,                 // structure/<key>
    pub name: Option<String>,
    pub kind: StructureKind,
    /// 通す道路と、その測点の範囲。2 本の車道を載せる 1 つの橋なら 2 件、
    /// 上下線別の 2 本のトンネルなら構造物を 2 つにして各 1 件。
    pub carries: Vec<StructureSpan>,     // { road, from: f64, to: f64 }
    /// 平面外形：床版の外形、坑道の平面投影。土地の扱い（§3.7）と
    /// 建物の除外（§4.9）に使う。
    pub outline: Footprint,
    pub site: Option<SiteId>,
}

pub enum StructureKind {
    Bridge {
        form: BridgeForm,                // Bridge | Viaduct | Overpass
        material: StructureMaterial,     // Concrete | Steel | Masonry | Timber
        deck_depth: f64,                 // 路面から桁下までの厚さ、m
        /// 支点：橋台（両端）と橋脚。地物ではなく構造物の幾何
        /// （Tin の三角形と同じ扱い）。
        supports: Vec<Support>,          // { kind: Abutment | Pier, footprint, top, base_z }
    },
    Tunnel {
        form: TunnelForm,                // Bored | CutAndCover | Underpass
        clear_height: f64,               // 路面から天井までの内空高、m
        lining: f64,                     // 覆工の厚さ、m（土被りの検査に使う）
        lighting: bool,
        /// 坑口：測点と坑門の幾何。
        portals: [Portal; 2],
    },
}
```

- `BridgeForm` の 3 値は OSM の `bridge=yes|viaduct` と、GDF の Brunnel Type（bridge / viaduct）に対応する。`Overpass` は他の道路をまたぐだけの短い橋。
- `TunnelForm` は OpenDRIVE の `standard | underpass` を細かくしたもの。`Bored`（山岳トンネル）と `CutAndCover`（開削）が `standard` に、`Underpass`（アンダーパス）が `underpass` に対応する。
- `material` は OpenDRIVE の `<bridge type="concrete|steel|brick|wood">` に対応する。
- 橋脚の位置は IR に持つ。CARLA の当たり判定、点群、LiDAR シミュレーションが橋脚を必要とするためで、生成器（§4.6）が決めて IR に書く。

### 3.2 `Road.structure` — 道路の側の属性

```rust
pub struct Road {
    // …既存…
    pub form: FormOfWay,
    /// どの区間がどの構造物の上（中）にあるか。範囲外は地表。
    pub structure: StructureProfile,     // Vec<(from, to, StructureId)>
}
```

`PavementProfile` と同じ「測点の範囲 → 値」の形にする。範囲どうしは重ならず、範囲外は「地表」である（土工を含む）。

### 3.3 `GradeSeparation` — 立体交差（関係）

```rust
/// 「over が under の上を via を通って越える」。GDF の
/// "X over Y through Brunnel" に当たる。
pub struct GradeSeparation {
    pub id: GradeSeparationId,           // separation/<over>/<under>/<n>
    pub over: RoadSpan,                  // { road, from, to }
    pub under: Underneath,               // Road(RoadSpan) | LandCover(LandCoverId)
    /// 上が橋（下の道路をまたぐ）なら橋、下がトンネル・アンダーパスならトンネル。
    pub via: StructureId,
}
```

- 関係を地物として持つ（README の "relationships as first-class objects"）。消費側は幾何を交差判定し直さなくても、「この道路はどこで何の下をくぐるか」を引ける。
- 下を通るのは道路か土地被覆（川など）である。谷を渡る高架のように、下に何もない場合は `GradeSeparation` を作らない。
- 実際の離隔（桁下と下の道路の高さの差）は **持たない**。幾何から計算でき、`validate()` と生成器が計算する。

### 3.4 `FormOfWay` — 道の形態

```rust
pub enum FormOfWay {
    /// 既定値。従来の地図はすべてこれになる。
    Unspecified,
    Motorway,            // 自動車専用で出入りが制限された本線の車道（上下線別なら片側 1 本）
    MultipleCarriageway, // 分離された車道を持つ一般道
    SingleCarriageway,
    SlipRoad,            // ランプ（出入路・連絡路）
    ParallelRoad,        // 集散路（C-D 路）・側道
    ServiceRoad,
    Roundabout,
}
```

- `RoadType`（OpenDRIVE の `<type>` 由来）は残す。`RoadType` は「どういう地域の道か」（`location`、速度の既定値）を表し、`FormOfWay` は「道路網の中で何の役割か」を表す。両者は別の軸である。
- 出力処理はこの値を使う：OSM の `highway=motorway_link`、SUMO の優先度、GPUDrive の `LaneFreeway`。
- **出入り制限**：`Motorway` と `SlipRoad` は平面交差を持てず、沿道に建物の間口を持たない。前者は GDF の MOTORWAY の定義どおり。検証（§3.9）と建物の配置（§4.9）がこれを使う。

### 3.5 `Interchange` — インターチェンジ（Level 2）

```rust
/// 道路どうしを立体的につなぐ複合地物。GDF の Level 2 Intersection に当たる。
pub struct Interchange {
    pub id: InterchangeId,               // interchange/<key>
    pub name: Option<String>,
    pub kind: String,                    // diamond / trumpet / cloverleaf / parclo / directional / …
    pub junctions: Vec<JunctionId>,
    pub roads: Vec<RoadId>,              // ランプ・加減速車線を含む本線の区間・集散路
    pub structures: Vec<StructureId>,
}
```

`Site` が「場所」をまとめるのに対し、`Interchange` は道路網の部品（交差点・ランプ・構造物）をまとめる。`kind` は `Site.kind` と同じく自由な文字列とし、カタログは持たない。構成要素の側は `Junction.interchange: Option<InterchangeId>` で参照し返す。

### 3.6 回廊の `Site`

高速道路の回廊（本線、法面、中央分離帯の土地被覆）は `Site { kind: "motorway" }` でまとめる。鉱山の Site と同じ仕組みで、新しい型は要らない。

### 3.7 地形 — 2.5D の TIN をそのまま使う

| 場所 | 地面（TIN） | 道路の縁の折れ線 | `Base::Roads` の高さの標本 |
| --- | --- | --- | --- |
| 地表の区間（土工を含む） | 道路の面に一致 | 入れる（従来どおり） | 使う |
| 橋・高架の区間 | 橋の **下** の地形（谷・川・下の道路） | 入れない | 使わない |
| トンネルの区間 | トンネルの **上** の地形（山・街） | 入れない | 使わない |
| 橋台 | 橋の端で地表に戻る | 橋の端に `abutment` の折れ線 | 使う |
| 坑口 | 坑門から外が切土 | 坑口に `portal` の折れ線（坑門の上端の高さ） | 使う |

新しい折れ線の種類：`fill_toe`（盛土の法尻）、`cut_crest`（切土の法肩）、`abutment`、`portal`、`bank` / `bed`（川岸・川底）。

新しい土地被覆の種類（自由文字列）：`embankment`（盛土の法面）、`cutting`（切土の法面）、`median`（中央分離帯）、`verge`（路肩の外の用地）、`water`（川）。

道路の平面外形で土地を切り抜く処理（CARLA の `terrain.rs`、`land.rs` の `Roads::near`）は、**地表の区間だけ** を対象にする。橋の下とトンネルの上には、周辺の格子点を置いてよい。

### 3.8 ID

| 地物 | ID | 例 |
| --- | --- | --- |
| 構造物（名前あり） | `structure/<slug(name)>` | `structure/arakawa_bridge` |
| 構造物（自動生成） | `structure/<road>/<kind>/<n>`（n は測点順） | `structure/e1_north/viaduct/0` |
| 立体交差 | `separation/<over>/<under>/<n>` | `separation/e1_north/avenue/0` |
| インターチェンジ | `interchange/<slug(name)>` | `interchange/kita_ic` |
| 折れ線 | `breakline/land/<structure>/<kind>/<n>` | `breakline/land/arakawa_bridge/abutment/0` |

鉱山と同じく、ID は入力から決まり、カウンタは使わない。新しいアリーナが空なら IR ダンプに出さないので、既存の地図の fingerprint は変わらない。

### 3.9 不変条件と検証（`validate()`）

| 検査 | 内容 |
| --- | --- |
| 参照の双方向性 | `Structure.carries` と `Road.structure` が一致する。`Interchange` と `Junction.interchange`、`GradeSeparation.via` が実在する |
| 区間 | `StructureProfile` の範囲が道路の範囲内にあり、互いに重ならない。1 つの構造物の `carries` が、その構造物の平面外形に収まる |
| 構造物の幾何 | 橋：支点の天端が桁下に、基部が地形（TIN があれば）に接する。トンネル：坑口が通す区間の両端にある |
| 土被り | TIN があるとき、トンネルの区間では「路面＋内空高＋覆工」が TIN より下にある（坑口の前後 `lining` 以内は除く）。TIN を突き抜けるトンネルはエラー |
| 桁下 | TIN があるとき、橋の区間では「路面−桁厚」が TIN より上にある。橋台の前後は除く。地面が床版を突き抜ける橋はエラー |
| **平面交差の禁止** | 交差点を共有しない 2 本の道路が平面で交差し、どちらかが (a) 構造物の上（中）にあるか、(b) `form` が `Motorway` / `SlipRoad` であるとき、上の道路の桁下と下の道路の路面の差が **物理的な最小値**（上の構造物の桁厚、トンネルなら覆工）以上であり、交差箇所を覆う `GradeSeparation` があること |
| 縦断 | 勾配の上限（`Motorway` / `SlipRoad` は 12%、それ以外は従来どおり無制限）。物理的にありえない値だけを拒否する |

「平面交差の禁止」は (a)(b) に当てはまる道路だけに適用する。既存の地図（web デモの "A town, generated" のように、同じ高さで交差点なしに交差するもの）はそのまま通る。法令上の建築限界（4.5 m など）は IR の不変条件ではなく、生成規則（§4）で守る。

## 4. 生成規則

### 4.1 段の順序

```
MapBuilder: add_road / add_junction / add_bridge / add_tunnel / connect_lanes …
   │          （道路の形と、利用者が明示した構造物）
MapBuilder::finish()
   │
environment::generate(&mut UnvalidatedMap, &Places)
   1. 自然地形      丘・川・周辺の起伏。道路によらない地面（§4.3）
   2. 構造物の判定  回廊の道路を自然地形と比べ、橋・高架・トンネル・土工に分ける（§4.4）
                    利用者が明示した構造物は変えない
   3. 立体交差      平面交差を検出し、橋（跨道橋）か GradeSeparation を作る（§4.5）
   4. 構造物の幾何  橋脚・橋台・坑口・床版外形（§4.6、§4.7）
   5. 土工          法面の折れ線（法尻・法肩）
   6. 建物          出入り制限のある道路と構造物を避ける（§4.9）
   7. 地形 TIN      §3.7 の表どおり、1 回だけ作る
   8. 土地被覆、Site、Interchange
   │
validate()
```

鉱山の `generate` と同じ関数を拡張する。`Places` に `corridors`、`structures`（明示分は builder 側）、`hills`、`rivers`、`interchanges` を足す。

### 4.2 設計基準（既定値）

道路構造令を参考にした既定値で、`DesignStandard` として差し替えられる。数値は実装前に原典と照合する（§11 Q6）。

| 設計速度 km/h | 120 | 100 | 80 | 60 | 40（ランプ） |
| --- | --- | --- | --- | --- | --- |
| 最小曲線半径 m | 710 | 460 | 280 | 150 | 60 |
| 最急縦断勾配 | 2% | 3% | 4% | 5% | 7% |
| 縦断曲線半径 凸 / 凹 m | 11000 / 4000 | 6500 / 3000 | 3000 / 2000 | 1400 / 1000 | 450 / 450 |
| 車線幅 m | 3.75 | 3.5 | 3.5 | 3.5 | 3.5 |
| 左路肩 / 右路肩 m | 2.5 / 1.25 | 2.5 / 1.25 | 2.5 / 0.75 | 1.75 / 0.75 | 1.5 / 0.75 |
| 中央分離帯 m | 4.5 | 4.5 | 3.0 | 3.0 | — |

共通の既定値：建築限界の高さ 4.5 m、跨道橋の桁厚 1.8 m、高架の桁厚 2.5 m、橋脚の間隔 40 m、トンネルの覆工 0.6 m、トンネルの内空高 4.5 m＋余裕 0.5 m、盛土の法面勾配 1:1.8、切土 1:1.2。

`DesignStandard` は 2 つのことに使う。

- 補助関数が返す点列と車線構成を作る。
- 利用者が描いた道路を **診断** する。基準から外れた道路は拒否せず、`Report.notes` に書く。基準外の道路も描けるようにするため。

### 4.3 自然地形と地形の部品

トンネルを通す山と、橋を架ける川が要る。いずれも地形の部品で、道路を作らない。

- **丘・尾根** `HillSpec { name, crest: Vec<Point3>, width, height }`：稜線（1 点なら円い丘）から両側へ、滑らかな断面（cos² の形）で下る。盛土（`HeapSpec`）の円錐台と違って法肩がないので、折れ線は作らず、自由点の高さだけを変える。土地被覆は作らない。
- **川** `RiverSpec { name, centre: Vec<Point3>, width, depth, bank_slope }`：川底と岸を折れ線（`bed` / `bank`）にした台形の溝。土地被覆は `water`。
- **自然地形の高さ**：`Base::Natural` を新設する。「回廊に属さない道路（街路など）の高さ」「部品のアンカー」「丘・川」から作り、**回廊の道路は標本に使わない**。回廊は地形に合わせて土工・構造物で通すものだからである。回廊のない地図では、従来の `Base::Roads` と同じになる。

### 4.4 構造物の判定（回廊の道路）

回廊に属する道路（`add_road(corridor=...)`）の各測点で、路面の高さと自然地形の高さの差 `d = 路面 − 自然地形` を取る。

| 条件 | 判定 |
| --- | --- |
| `d > H_bridge`（既定 8 m）が `L_min`（既定 30 m）以上続く | 高架（`Viaduct`）。長さ 100 m 未満なら `Bridge` |
| 区間内に川（`water`）がある | 橋（`Bridge`）。川幅＋両岸 10 m を最低限とし、上の規則の区間と合わせる |
| `d < −(内空高 + 覆工 + C_min)`（`C_min` は最小土被り、既定 10 m）が `L_min` 以上続く | 山岳トンネル（`Bored`） |
| 市街地（§4.9 の街区の範囲）で `d < −(内空高 + 覆工 + 2 m)` | 開削トンネル（`CutAndCover`） |
| 上記以外 | 土工。盛土（`d > 0`）または切土（`d < 0`）の法面を折れ線にする |

- 判定した区間は前後に延ばす。橋台・坑口は「`d` が `H_bridge` / 土被りの閾値を下回る点」ではなく、法面の高さが `H_portal`（既定 12 m、坑門の高さ＋α）以下になる点に置く。延長が不自然に長い切土・盛土を作らないため。
- 構造物どうしの間が `L_gap`（既定 50 m）未満なら 1 つにまとめる。短い土工区間を挟んで構造物が細切れになるのを防ぐ。
- 上下線の 2 本の車道（同じ回廊）は、判定を **合わせる**。高架なら 1 つの構造物が 2 本を載せ、山岳トンネルなら上下線で別の構造物（2 本のトンネル）にする。開削トンネルは 1 つにする。
- 利用者が `add_bridge` / `add_tunnel` で明示した区間は判定しない。回廊に `structures="explicit"` を指定すると、自動判定をすべて止める。

### 4.5 立体交差（すべての道路）

交差点を共有しない 2 本の道路の平面交差を、`Segments` インデックスで検出する。高さの差を `Δz`、上の道路の桁厚を `t` として：

| 条件 | 処理 |
| --- | --- |
| 交差箇所で上の道路が既に構造物（橋）の区間にある | 下の道路（または川）を `under` とする `GradeSeparation` を作る |
| 下の道路が既にトンネルの区間にある | `via` をそのトンネルとする `GradeSeparation` を作る |
| `Δz ≥ 4.5 + t` で、どちらにも構造物がない | 上の道路に跨道橋（`Overpass`）を作る。下の道路の全幅＋両側の余裕（既定 2 m）をまたぎ、両端に橋台を置く。続けて `GradeSeparation` を作る |
| `t ≤ Δz < 4.5 + t` | 構造物は作り、`Report.notes` に「建築限界不足」と書く |
| `Δz < t`、どちらかが §3.9 (a)(b) に当てはまる | エラー（生成時。`validate()` も同じ条件で拒否する） |
| `Δz < t`、どちらも当てはまらない | 何もしない（従来の、交差点なしの平面交差） |

**生成器は道路の高さを変えない。** 街路を高速道路の上に通すには、利用者が街路の縦断を上げる必要がある。そのための補助関数 `Corridor.crossing(points, over=True)` は、回廊を建築限界以上の高さで越えるよう、縦断曲線付きで z を上げた点列を返す（§5.2）。

### 4.6 橋・高架の幾何

- **床版外形**：通す道路の外側の縁に地覆（既定 0.6 m）を足したもの。上下線を載せる高架は、中央分離帯を含めて 1 枚にする。
- **橋台**：構造物の両端。幅は床版、奥行 6 m。基部は地形の高さ。
- **橋脚**：両端から等間隔に `pier_spacing`（既定 40 m）以下で置く。ただし、下の道路の **車道と路肩の上** には置かない。置けない位置の橋脚は前後にずらし、ずらしてもスパンが 1.5 倍を超える場合はそこで支間を割り直す。川の中には置いてよいが、`Report.notes` に書く。
- 橋脚の断面は、床版の幅が 15 m 以下なら単柱、それ以上なら 2 本柱（`Support.footprint` は柱ごとに 1 つ）。
- 高架では地覆の外側に壁高欄（`Barrier { kind: "parapet", height: 1.1 }`）を自動で置く。

### 4.7 トンネルの幾何

- **坑口**：通す区間の両端。坑門の幾何は、路面の高さから内空高＋覆工まで立ち上がる面（`Portal { station, face: Vec<Point3> }`）である。
- **平面外形**：通す道路の外縁＋監視員通路（既定 0.75 m）＋覆工。
- 照明：`lighting = true` を既定とする。延長 50 m 未満のアンダーパスは `false`。
- トンネル内の路肩は、設計基準の値ではなく狭い値（既定：左 0.75 m）に切り替えるのが普通である。これは車線構成の話なので、補助関数（`Corridor.carriageway()` が返す車線構成）が坑口の前後に `cross_sections` を入れて切り替える。生成器は道路を変えない。

### 4.8 インターチェンジの規則

補助関数 `Corridor.interchange(kind, at, crossroad, …)` は、点列・車線構成・接続の組である `InterchangePlan` を返す。利用者はそれを `m.add_interchange(plan)` で地図に入れるか、点列を書き換えてから入れる。`plan` を使わず、ランプを手で描いて `m.add_interchange(name, kind, junctions, roads)` でまとめるだけでもよい。

| 形式 | 用途 | 生成する部品 |
| --- | --- | --- |
| `diamond` | 高速道路と市街地の幹線（信号で処理できる交通量） | ランプ 4 本、交差道路側に平面交差 2 か所（信号） |
| `half_diamond` | 片方向だけの出入り（都市部、用地が狭い） | ランプ 2 本 |
| `parclo` | 交差道路の交通量が多い | ループランプ 2 本＋ランプ 2 本 |
| `trumpet` | 高速道路と有料道路（料金所の位置）・T 字の接続 | ループ 1＋準直結 3 |
| `cloverleaf` | 高速道路どうし（交通量が少ない） | ループ 4＋外回り 4、集散路 |
| `directional` | 高速道路どうし（JCT） | 直結ランプ 8。多層の高架 |

共通の規則：

1. **加減速車線**：本線にテーパー付きの車線を `cross_sections` で足す。長さは本線とランプの設計速度による（既定：減速車線 本線 100 km/h で 90 m＋テーパー 60 m、加速車線 本線 100 km/h で 180 m＋テーパー 60 m。要確認、§11 Q6）。ランプは減速車線の **分流ノーズ** から始まり、合流ノーズで本線の加速車線へ接続する（`connect_lanes`）。
2. **ランプの線形**：ランプの設計速度（既定 40 km/h、ループは 30–40 km/h）の最小半径・最急勾配（§4.2）に従う。本線との高さの差は縦断曲線付きの一定勾配で解消する。
3. **ランプの構造物**：ランプにも §4.4 の判定を適用する。直結ランプの高架や、ループと本線の交差は §4.5 で処理される。
4. **ランプ端の交差点**（`diamond` / `parclo` の一般道側）：平面交差（`add_junction`）として作り、信号（`add_traffic_light` と `add_traffic_light_rule`）を既定で付ける。`control="yield"` なら一時停止と優先関係。出口ランプの端には停止線を置く。
5. **出入り口の向き**：右側通行（`handedness="rht"`）では本線の右から出入りし、左側通行（`lht`、日本）では左から出入りする。地図の `handedness` に従う。
6. **間隔**：同じ回廊でインターチェンジどうしの距離（分流ノーズから次の合流ノーズまで）が `min_weave`（既定 600 m）を下回れば、`Report.notes` に「織り込み区間が短い」と書く。

### 4.9 高速道路と市街地をつなぐ規則

#### (1) 建物

- **間口**：建物の区画は、`form` が `Motorway` / `SlipRoad` の道路、構造物の区間、回廊の道路には作らない（出入り制限）。
- **除外範囲**：建物の外形は次と重ねない。
  - 地表の回廊：道路の外縁＋法面＋用地幅（既定 10 m）
  - 橋・高架：床版外形＋5 m（高架の下に建物を建てない）
  - 開削トンネル：外形の真上（土被りが浅いため）
- 山岳トンネルの上には建ててよい。
- 街区の範囲：`Places.town` の対象道路から `Layout.setback + lot_depth` 以内を「市街地」とする。§4.4 の開削トンネルの判定に使う。

#### (2) 市街地の上を通る（都市高速）

回廊の一部が市街地の範囲を通るとき：

- 自動判定（§4.4）の結果によらず、その区間は **高架** を既定にする（`urban="elevated"`）。`urban="tunnel"` なら開削トンネル（地下式）にする。
- 街路との交差はすべて §4.5 で `GradeSeparation` になる。橋脚は街路の車道に置かない（§4.6）。橋脚を街路の中央分離帯に置く場合は、利用者が中央分離帯を `type_="border"` の車線として描けば置ける。
- 出入口ランプ（`Corridor.urban_ramp(street, at, direction)`）は、街路の中央または側方から最急勾配（既定 7%）で高架まで上がる `SlipRoad` の点列を返す。街路の側では §4.8 (4) と同じく信号付きの平面交差でつなぐ。

#### (3) 高速道路の終点から一般道へ

本線をそのまま一般道に続けるとき（`Corridor.terminate(into=street, at)`）：

1. 移行区間（既定 400 m）で、設計速度を段階的に落とす（100 → 80 → 60 km/h）。`add_speed_limit` を区間ごとに付ける。
2. `form` を `Motorway` から `MultipleCarriageway` に切り替える。道路を分けて 2 本にし、境界に交差点は置かない（`connect`）。
3. 車線数が減るなら、`cross_sections` とテーパーで減らす。
4. 一般道に入った最初の交差点には信号を付ける。
5. 一般道の区間からは建物の間口を作れるようになる（(1) の除外がここで終わる）。

#### (4) 市街地の街路が高速道路を横切る

- 跨道橋：利用者が `Corridor.crossing(points, over=True)` で上げた街路の点列を描けば、§4.5 で `Overpass` が付く。
- アンダーパス：`over=False` なら、街路を下げた点列が返る。高速道路側には、街路の幅をまたぐ `Overpass` が付く。
- 平面交差（同じ高さで交差点なし）は §3.9 でエラーになる。

### 4.10 決定性

同じ `Places`（seed を含む）からは常に同じ地図ができる。構造物の判定は測点（`sampling` の間隔）ごとに決定的に行う。

## 5. API

### 5.1 Rust

```rust
// roadgen-core
MapBuilder::add_bridge(&mut self, spans: &[RoadSpan], spec: BridgeSpec) -> StructureId
MapBuilder::add_tunnel(&mut self, spans: &[RoadSpan], spec: TunnelSpec) -> StructureId
RoadSpec::with_form(FormOfWay)
RoadSpec::with_vertical(VerticalProfile)      // 縦断曲線（§5.3）

// roadgen-carla::environment
pub struct Places {
    pub pits, heaps, pads, site_roads, town, ground, seed,   // 既存
    pub corridors: Vec<CorridorSpec>,
    pub corridor_roads: Vec<(RoadId, String)>,
    pub hills: Vec<HillSpec>,
    pub rivers: Vec<RiverSpec>,
    pub interchanges: Vec<InterchangeSpec>,   // 名前・形式・構成要素
}
CorridorSpec::carriageway(&self, side, standard) -> (Vec<Point3>, Vec<LaneSpec>, Vec<CrossSectionSpec>)
CorridorSpec::interchange(&self, kind, at, crossroad) -> InterchangePlan
```

### 5.2 Python

```python
m = roadgen.Map(name="expressway", handedness="lht")

# 地形の部品
m.add_hill(name="Kita Ridge", crest=[(800.0, -600.0, 0.0), (900.0, 600.0, 0.0)],
           width=500.0, height=70.0)
m.add_river(name="Ara River", centre=[(-400.0, -800.0, 0.0), (-350.0, 800.0, 0.0)],
            width=90.0, depth=6.0)

# 回廊は道路を作らない。縦断（PVI と縦断曲線）と構造物の規則を持つ部品
e1 = m.add_corridor(name="E1", centre=[(-1500.0, 0.0, 12.0), (2000.0, 0.0, 30.0)],
                    design_speed=100, profile=[(0.0, 12.0), (1200.0, 18.0), (3500.0, 30.0)],
                    structures="auto", urban="elevated")
# 上下線の点列と車線構成は補助関数が返す。点列を書き換えてもよい
for side in ("left", "right"):
    c = e1.carriageway(side, lanes=2)
    m.add_road(points=c.points, lanes=c.lanes, cross_sections=c.cross_sections,
               name=f"e1_{side}", type_="motorway", form="motorway", corridor="E1")

# 明示した構造物（自動判定より優先）
m.add_bridge(["e1_left", "e1_right"], s=(400.0, 620.0), form="bridge",
             material="steel", name="Arakawa Bridge")

# インターチェンジ：計画を受け取り、確認・編集してから地図に入れる
plan = e1.interchange("diamond", at=1800.0, crossroad="avenue")
m.add_interchange(plan, name="Kita IC")

# 街路を跨道橋にする
street = e1.crossing([(300.0, -400.0, 0.0), (300.0, 400.0, 0.0)], over=True)
m.add_road(points=street, lanes=two_way(), name="bridge_street")

m.generate_buildings(rules="town")       # 回廊・高架の下・ランプは自動で避ける
m.validate()
m.export_carla("Import/", name="Expressway")

m.structure_ids()                         # ['structure/arakawa_bridge', 'structure/e1_left/tunnel/0', …]
m.structure("structure/arakawa_bridge")   # {"kind": "bridge", "carries": [...], "supports": [...]}
m.grade_separations()                     # [{"over": ..., "under": ..., "via": ...}]
m.interchange("interchange/kita_ic")
```

回廊を使わず、全部を手で描くこともできる。`add_road(form="slip_road")`、`connect_lanes`、`add_bridge` / `add_tunnel`、`add_interchange(name, kind, junctions=[...], roads=[...])` を使う。この場合も、立体交差の検出（§4.5）・地形・建物の規則は同じに働く。

### 5.3 縦断曲線

`VerticalProfile { pvis: Vec<(station, z)>, curves: Vec<f64 /*縦断曲線長 or 半径*/> }`。PVI の間を一定勾配で結び、各 PVI に放物線の縦断曲線を入れる。IR の `Curve3` は放物線の z を持てないので、builder は平面線形の各ピースを `sampling` 以下の長さに分割し、各小区間を一定勾配で近似する（半径 3000 m の曲線を 2 m で分割した場合、誤差は 0.2 mm 未満）。OpenDRIVE の書き出しは既に勾配ごとに `<elevation>` を書くので、変更は要らない。

## 6. 各出力処理

| 形式 | 構造物 | 立体交差 | `FormOfWay` | 地形・被覆 |
| --- | --- | --- | --- | --- |
| OpenDRIVE | `<objects><bridge s length type>` / `<tunnel s length type lighting daylight>`。**読み戻しも実装する**（範囲 → `Structure`、外形は道路の縁から再構成し、橋脚は失われると注記） | 書かない（3D の高さで表れる） | 書けない。`<type>` のみ。`check()` が注記 | 書かない（従来どおり） |
| OSM | way を構造物の境界で分割し、`bridge=yes|viaduct`、`tunnel=yes`（アンダーパスは `tunnel=yes`、開削も同じ）、`layer=±n` を付ける。床版は `man_made=bridge` の面 | `layer` を下から順に導出する（地表 0、橋 +1、橋の上の橋 +2、トンネル −1） | `Motorway` → `highway=motorway`、`SlipRoad` → `motorway_link`（本線が motorway のとき）、分流点の節点に `highway=motorway_junction` | `natural=water`（川） |
| Lanelet2 | 形式に該当するものがない。z だけで区別できる。`check()` が注記 | 同上 | `subtype=highway`（`Motorway` / `SlipRoad`） | 書かない |
| SUMO | 書かない（z は残る）。`check()` が注記 | 節点を共有しないので、そのままで正しい | 優先度に反映（`Motorway` 13、`SlipRoad` 11） | 書かない |
| ClipGT | 書かない（z は残る） | 同上 | 書かない | 書かない |
| GPUDrive | 2D なので、**上の道路の `road_edge` のうち、下の道路の外形と平面で重なる部分を切る**。`check()` が注記 | 上記の処理 | `Motorway` / `SlipRoad` → `LaneFreeway` | 書かない |
| CARLA | §7 | — | — | TIN から（§3.7） |
| viewer | 橋は縁取り（ケーシング）付き、トンネルは破線で描く。描画順は `layer` の順 | — | — | — |

## 7. CARLA のアセット

| IR | アセット |
| --- | --- |
| 橋の区間の道路 | 路面のリボンに **床版の裏面と側面** を足す（桁厚 `deck_depth`）。高欄は `Barrier`（壁）として既存の処理で描く |
| 橋脚・橋台 | `Support.footprint` を基部から天端まで押し出した柱、橋台は台形の塊。材質はコンクリート |
| トンネルの区間 | 路面に加え、内壁・天井（`clear_height`）・覆工の外面。坑門（`Portal.face`）の面と、その周りの切土の面 |
| 地面 | TIN から作る。橋の下は地面が続き、トンネルの上も地面が続く |
| TIN がない地図 | `ground::Field` の高さの標本から構造物の区間の道路を除く。土地の切り抜きも地表の区間だけにする |
| 小物（標識・信号） | 床版の上では高欄の内側に置く（歩道のない道路で、柱が空中に出るのを防ぐ） |
| 点群 | 床版の裏面・橋脚・トンネルの内壁が三角形として入るので、そのまま点群に出る |

材質の追加：`M_Concrete_Structure`（床版・橋脚・坑門）、`M_Tunnel_Lining`。Poly Haven のスラッグは実装時に決める。

## 8. テスト

| 種類 | テスト |
| --- | --- |
| core 検証 | 構造物と道路の参照の不一致、範囲の重なり、平面交差の禁止（motorway / 構造物 / 従来の街路の 3 通り）、土被り不足、地面が床版を突き抜ける |
| 生成 | 丘を貫く道路がトンネルになる。川を渡る道路が橋になる。谷を渡る道路が高架になる。上下線の判定が揃う。短い土工区間を挟む構造物がまとまる。橋脚が下の道路の車道に立たない |
| 立体交差 | 街路を跨道橋にすると `Overpass` と `GradeSeparation` ができる。建築限界の不足が notes に出る |
| インターチェンジ | `diamond` の計画から作った地図が検証を通り、ランプ端に信号が付く。`lht` で左から出入りする |
| 市街地 | 高架の下に建物が建たない。終点の移行区間で速度規制が段階的に変わり、一般道の区間からは建物が建つ |
| 出力 | OpenDRIVE の `<bridge>` / `<tunnel>` の往復。OSM の `layer` / `bridge` / `motorway_link`。GPUDrive の `road_edge` の切断。CARLA に床版の裏面・橋脚・坑門のメッシュが出る |
| 互換 | 既存の地図の IR fingerprint が変わらない。web デモの全例が動く |
| 統合 | 参照用の高速道路（丘のトンネル、川の橋、市街地の高架、diamond IC、終点の移行区間）を 1 枚の地図で作り、全形式に出力する |

## 9. 実装の段階（PR の分け方）

| PR | 内容 | 依存 |
| --- | --- | --- |
| 1. IR | `Structure` / `StructureProfile` / `GradeSeparation` / `FormOfWay` / `Interchange`、検証、IR ダンプ、`add_bridge` / `add_tunnel` / `with_form`、Python の読み書き | — |
| 2. 出力 | OpenDRIVE（書き出し・読み戻し）、OSM、Lanelet2 / SUMO / GPUDrive、viewer | 1 |
| 3. 地形と CARLA | 構造物の区間を除いた土地、自然地形（`Base::Natural`、丘・川）、床版・橋脚・トンネル・坑門のメッシュ | 1 |
| 4. 生成規則 | 縦断曲線、回廊、構造物の自動判定、立体交差の検出、橋脚の配置、建物の除外 | 1, 3 |
| 5. インターチェンジと市街地 | `InterchangePlan`（diamond / half_diamond / parclo / trumpet から。cloverleaf / directional は後でもよい）、都市高速のランプ、終点の移行、web デモ、README、`docs/highway-design.md` の実装状況 | 4 |

PR 1 と 2 だけでも、利用者が手で描いた道路に橋・トンネルを明示すれば全形式に出る。生成規則（PR 4・5）はその上に載る。

## 10. 既知の制約・今後

- 鉄道・水路の橋、歩道橋は範囲外。`Underneath` に `Railway` を足せば拡張できる。
- 料金所（GDF の Toll、OSM の `barrier=toll_booth`）、SA / PA（GDF の Enclosed Traffic Area）は今回入れない。SA / PA は `Site { kind: "service_area" }` ＋ `add_pad` で、鉱山のヤードと同じ形で作れる見込み。
- 斜張橋・吊橋などの上部構造の形は表現しない（桁橋として描く）。
- トンネルの断面は箱形（壁＋平らな天井）で近似する。円形・馬蹄形は今後。
- 構造物の区間で車線数が変わる場合も扱えるが、床版の外形は区間ごとの最大幅で 1 枚にする。

## 11. レビューで決めてほしい点

| # | 論点 | 提案 | 代案 |
| --- | --- | --- | --- |
| Q1 | 道路の属性と構造物地物の **両方** を持つか | 両方（GDF / MultiNet と同じ。双方向の参照を検証する） | 構造物地物だけにして、道路の側は導出する（重複はないが、道路を読むたびに逆引きが要る） |
| Q2 | 相対レベル（layer）を IR に持つか | 持たない。3D 座標から出力処理が導出する | 持つ（OSM の `layer` を利用者が指定できるが、z と食い違いうる） |
| Q3 | 平面交差の禁止の範囲 | `Motorway` / `SlipRoad` / 構造物の区間に限る（既存の地図は通る） | 全道路に適用する（web デモの "A town" を直す必要がある。破壊的変更） |
| Q4 | 回廊が道路を作らない（鉱山のピットと同じ）か | 作らない。補助関数が点列と車線を返し、利用者が `add_road` する | `add_motorway()` が上下線の道路まで作る（手数は減るが、道路の形を利用者が決めるという原則から外れる） |
| Q5 | 生成器が利用者の道路の高さを変えるか | 変えない。跨道橋などの高さは補助関数が点列で返す | 立体交差の検出時に、下の道路を自動で下げる・上の道路を上げる |
| Q6 | 設計基準の既定値 | 道路構造令を参考にした §4.2 の表（実装前に原典と照合する。加減速車線長は特に要確認） | 国によらない値（AASHTO など）、または既定値を持たず必須引数にする |
| Q7 | `FormOfWay` を `RoadType` と別に持つか | 別に持つ（地域の分類と網の中の役割は別の軸） | `RoadType` に `MotorwayLink` などを足す（OpenDRIVE の `<type>` の語彙から外れる） |
| Q8 | インターチェンジの形式をどこまで最初に入れるか | diamond / half_diamond / parclo / trumpet。cloverleaf / directional は PR 5 の後半か別 PR | 全部を一度に入れる |
| Q9 | 都市部の回廊の既定 | 高架（`urban="elevated"`） | 地表（市街地でも自動判定だけに任せる） |
