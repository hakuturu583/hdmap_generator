# Canonical Road IR の構造と、走行ログからの逆生成

このリポジトリが内部で使っている IR（`roadgen-core` の Canonical Road IR）の構造を
まとめ、続けて「実車の走行ログからこの IR のデータを起こす」にはどんなモデルを置き、
どんな情報を集めればよいかを分析する。

前半は現状のコードの記述、後半は設計メモであって実装ではない。後半に出てくるものは
このリポジトリにはまだ何も存在しない — roadgen は生成器であって変換器ではなく、
既存の地図やログを読む経路は今のところ一本もない。

---

## 1. IR の構造

### 1.1 全体像

`roadgen_core::map::Map` が IR そのもので、中身は型付き ID をキーにした
アリーナ（`Arena<Id, T>`）の集まりである。

| フィールド | 型 | 何を持つか |
| --- | --- | --- |
| `metadata` | `MapMetadata` | 原点・投影・左右通行・サンプリング刻み |
| `roads` | `Arena<RoadId, Road>` | 参照線を 1 本持つ道路の区間 |
| `lanes` | `Arena<LaneId, Lane>` | 1 本の道路の 1 本の車線 |
| `junctions` | `Arena<JunctionId, Junction>` | 交差点（流入道路とコネクタ道路の並び） |
| `connections` | `Arena<ConnectionId, LaneConnection>` | 車線から車線への移動の許可 |
| `objects` | `Arena<ObjectId, MapObject>` | 信号・標識・停止線・横断歩道 |
| `rules` | `Vec<TrafficRule>` | 信号制御・優先関係・車線単位の速度制限 |
| `buildings` / `building_parts` | `Arena<…>` | 沿道の建物と、その塊（massing） |

`MapMetadata` は `name`、`origin: GeoOrigin`（緯度・経度・標高）、
`projection`（`LocalCartesian` / `Utm` / `Mgrs`）、`handedness`（RHT / LHT）、
`sampling: SamplingConfig { max_segment_length }`。地図の x/y は常に「原点まわりのメートル」で、
`projection` はそれを何のフレームで読むかだけを決める。

### 1.2 4 つの分離

`roadgen-core` のモジュール構成がそのまま設計の主張になっている。

- **トポロジー**（`topology`）は接続だけを言う。座標を一切含まないので、
  ジオメトリが 1 点も存在しない状態で構築できる。
- **ジオメトリ**（`geometry`）は最初から 3 次元で、2D 型が存在しない。
  平面計算が要るところは `horizontal` という名前か `Frame3::to_local` を通る。
- **セマンティクス**（`semantics`）は意味だけを言い、ファイル形式に触れない。
- **物理エンコーディング**はエクスポータ側のクレートにあり、IR はそれらに依存しない。
  形式が表現できないものは IR のフィールドではなく、各エクスポータの `check()` の
  戻り値として呼び出し側に返る。

設計の系譜は GDF 5.1（特徴とトポロジーの分離、概念モデルと物理エンコーディングの分離、
関係を第一級オブジェクトにすること、安定な識別子）からアイデアだけを取っている。

### 1.3 トポロジー

座標を持たない語彙:

- `Direction::{Forward, Backward}` — 参照線に対する進行方向。
- `LateralSide::{Left, Right}` — 参照線のどちら側か（符号 +1 / −1）。
- `LaneEnd` / `RoadEnd` — `Start` / `End`。**意図的に entry/exit ではない**。
  どちらが入口かは `Direction` が決めるので、混同すると接続が裏返る。
- `LaneEndpoint { lane, end }` / `RoadEndpoint { road, end }`。
- `RoadLink { predecessor, successor }`、各々 `RoadLinkTarget::{Road(RoadEndpoint), Junction(JunctionId)}`。
- `LaneConnection { id, from: LaneEndpoint, to: LaneEndpoint, junction: Option<JunctionId> }`
  — 接続の唯一の正本。`from.end` は交通が出る側、`to.end` は入る側なので、
  2 本の参照線の向きが逆でも同じ読み方になる。
- `Junction { incoming_roads, connecting_roads }`。

IR は交差点を「点」としては描かない。アームは交差点の手前で止まり、
そこを埋める **コネクタ道路**（1 車線の生成された道路）が渡す。点を必要とする形式
（OSM のノード、SUMO のジャンクション）のために `Map::junction_centre()` が
各アーム中心線を延長した最小二乗交点を計算する。

### 1.4 ジオメトリ

すべての曲線は **水平ステーション `s`**（水平面に落とした影の弧長）で媒介変数表示される。
OpenDRIVE と同じパラメータで、標高は捨てずに持って回る。

```
Curve3 = Line(Line3) | Arc(Arc3) | Clothoid(Clothoid3) | Bezier(Bezier3)
       | Polyline(Polyline3) | Composite(Vec<Curve3>)
```

- `Arc3` / `Clothoid3` は平面曲率と一定勾配。クロソイドはフレネル積分なので
  0.01 m 刻みのシンプソン積分（16〜4096 区間）。
- `Bezier3` は解像度を曲線自身が持つ（`t` 媒介なので、歩幅が変わると長さが変わってしまうため）。
- `Composite` は端点が 1 mm 以内で繋がっていることを構築時に検査する。
- `Alignment` は「直線→緩和→円→緩和→直線」を、点・方位・曲率を引き継ぎながら
  積み上げるビルダーで、構成上滑らかになる。

沿線方向に変化する量は 2 種類:

- `Poly3Profile`（区分 3 次式 `a + b·ds + c·ds² + d·ds³`）— `Road::superelevation`（横断勾配、ラジアン）と
  `Road::lane_offset`（断面原点の参照線からの横ずれ）に使う。
- `WidthProfile` — `PositiveWidth` のノット列 + `Taper::{Linear, Smooth}`。
  補間が単調なので、**幅が 0 以下になる表現が存在しない**。自由係数の 3 次式を置かないのは
  このため（OpenDRIVE の `<width>` は一般の 3 次式なので途中で負に潜れる）。

### 1.5 道路と車線

```rust
struct Road {
    id, name,
    reference_line: Curve3,
    lane_offset: Poly3Profile,
    lanes: Vec<LaneId>,
    sections: Vec<CrossSection { station, lanes }>,  // 昇順、先頭は station 0
    junction: Option<JunctionId>,                    // Some ならコネクタ
    link: RoadLink,
    road_type: RoadType,
    speed_limit: Option<SpeedLimit>,
    superelevation: Poly3Profile,
}
```

車線数が途中で変わる道路は `CrossSection` を複数持つ。**単に細くなるだけなら新しい断面は不要**で、
それは幅プロファイルの仕事。

```rust
struct Lane {
    id, road, index, side, ordinal, direction, lane_type,
    width: WidthProfile, speed_limit,
    section: usize, station_range: (f64, f64),
    left_edge: i32, right_edge: i32,     // 断面原点から外向きに数えた「境界の番号」
    left_offset: f64, right_offset: f64, // 車線始端での横オフセット（参考値）
    left_boundary: Curve3, right_boundary: Curve3, centerline: Curve3,
    left_marking: BoundaryMarking, right_marking: BoundaryMarking,
}
```

要点が 2 つ。

1. 境界と中心線は **参照線順**で格納される（`left` は常に参照線の左）。
   進行方向で欲しいときは `travel_geometry()` を使う。逆走車線では曲線の反転と
   左右の入れ替えを**両方**やる必要があり、片方だけやるのが鏡像レーンレットの典型的な作り方。
2. `left_edge` / `right_edge` は整数。2 つの車線が境界を共有するのは
   **この整数が一致するとき、かつそのときに限る**。テーパで横オフセットが合わなくなっても
   共有関係は保たれ、これが Lanelet2 出力での境界共有の根拠になる。

### 1.6 セマンティクス

- `LaneType`: `Driving, Shoulder, Border, Sidewalk, Biking, Parking, Restricted, None`。
  `is_drivable()` は `Driving | Biking` のみ（レーンレットになるか、接続に参加するかを決める）。
- `RoadType`: `Town, Rural, Motorway, LowSpeed, Pedestrian`。
- `BoundaryMarking { marking: RoadMarking, color: MarkingColor }`。
  `RoadMarking` は `None, Solid, Broken, SolidSolid, BrokenSolid, SolidBroken, Curbstone`、
  色は白・黄。
- `MapObject { kind, geometry, lanes }`。
  `kind` は `TrafficLight | TrafficSign { code } | StopLine | Crosswalk`、
  `geometry` は `Point | Line | Band { left, right }`。
  標識コードのカタログは **IR が定義しない**（呼び出し側の体系をそのまま通す）。
- `TrafficRule`: `TrafficLight { lights, stop_line, lanes }` /
  `RightOfWay { right_of_way, yielding, stop_line }` / `SpeedLimit { limit, lanes }`。

### 1.7 建物

道路の意味論ではないので別のアリーナに置かれている。
`Building { parts, kind: String, frontage: Option<Frontage { road, side, station } > }`、
`BuildingPart { building, solid: Solid, kind, levels }`、
`Solid { footprint: Footprint, wall_height, roof: Roof { shape, height, direction } }`。
`Footprint` は開いたリング（先頭頂点を末尾で繰り返さない）で、上から見て反時計回りに正規化され、
**頂点ごとに z を持つ**（斜面の上の建物は傾いた輪郭を持つ）。
`Solid::shell()` は底面・壁のクワッド・屋根を返し、全辺がちょうど 2 面に共有される閉じた曲面になる。

### 1.8 識別子と検証ゲート

識別子はカウンタではなく入力から導かれる（`road/north`, `lane/north/0`,
`connection/j0/north_0/east_0`）。同じ入力からは常に同じ ID が出るので、
再生成して diff を取ることに意味がある。OpenDRIVE / Lanelet2 の ID はエクスポータが
IR の反復順から振り、IR には決して戻らない。

地図は `MapBuilder → UnvalidatedMap → validate() → ValidatedMap` と進み、
エクスポータは最後のものしか受け取らない。型で潰せる制約（幅 > 0、単位ベクトル、
2 頂点以上のポリライン、地球上の緯度経度）は検証では見ない。検証が見るのは
**組み合わせて初めて壊れるもの**: ID の一意性、ダングリング参照、車線と道路の整合、
道路リンクの相互性、リンクした道路が空間的に本当に接しているか、接続が交通の実際に使う端で
出入りしているか、接続車線の中心線と境界が XYZ で連続か、車線の境界が左右正しく幅を張っているか、
交差点の所属、座標メタデータ、建物の妥当性。

許容差は `ValidationConfig { position_tolerance: 1e-3, width_tolerance: 2e-2 }`。
**1 mm** である。これは後半の議論に効く。

---

## 2. 走行ログから IR を起こす

### 2.1 推定ターゲットを間違えないこと

素朴にやると「知覚で車線境界のポリラインを出して `Lane` を直接埋める」になるが、これは失敗する。

- 位置許容差が 1 mm。知覚由来のポリラインを 2 本並べて「境界を共有している」と主張しても、
  `left_edge`/`right_edge` の整数一致という IR の共有表現には乗らない。
- `Lane.centerline` / `left_boundary` / `right_boundary` は **ビルダーが導出する値**であって、
  独立の自由度ではない。参照線・断面・幅プロファイルから生成される。
- `Curve3::Polyline` を参照線に使えば一応通るが、OpenDRIVE 側の品質が落ちる
  （曲率が段々に不連続になり、ノイズがそのまま出力に出る）。

したがって **推定ターゲットは `Map` ではなく `MapBuilder` の入力**にする。
つまりログから起こすのは次の集合だけでよい:

```
RoadSpec { reference_line: Curve3(Alignment), cross_sections: [(station, [LaneSpec])],
           road_type, speed_limit, superelevation }
LaneSpec { width: WidthProfile, direction, lane_type, speed_limit, side, markings }
add_junction / connect_ends(from, from_end, to, to_end, junction) / connect_lanes
add_stop_line / add_traffic_light / add_traffic_sign / add_crosswalk
add_traffic_light_rule / add_right_of_way / add_speed_limit
```

これは「生成器を逆に解く」という定式化で、利点が 3 つある。
車線ジオメトリ・境界共有・コネクタ曲線は既存コードが作るので**構成上 `validate()` を通る**。
パラメータ数が桁違いに小さいので推定問題として素性がよい。
そして出力が Python API の呼び出し列（= 人間が読めるスクリプト）になるので、
手で直せるし diff も取れる。

交差点についても、コネクタの形を推定する必要はない。アームの終端と
「どの movement が存在するか」だけ与えれば、`connect_via` / `connect_lanes` が
端点接線を固定した 3 次曲線としてコネクタを生成する。

### 2.2 パイプラインの段

```
 走行ログ（複数パス）
   │
   ├─ 1. 位置推定      LiDAR-IMU オドメトリ + GNSS/RTK + ポーズグラフ、マルチセッション整合
   │                   → 全パス共通のメートル座標、GeoOrigin / projection
   ├─ 2. 集約          反射強度 BEV ラスタ（5–10 cm/px）+ 高さ + 画像セマンティクス投影
   ├─ 3. ベクトル抽出  車線境界・中心線・停止線・横断歩道のインスタンス + クラス/色
   ├─ 4. レーングラフ  境界対 → 車線、軌跡の遷移 → 接続、方向
   ├─ 5. 道路組み立て  並走車線のクラスタリング → Road、参照線フィット、断面分割、幅プロファイル
   ├─ 6. 交差点抽出    区画検出、アームの切り戻し、movement の列挙
   ├─ 7. 物体と規則    信号・標識の三角測量、規則の挙動推論
   └─ 8. IR 当てはめ   RoadSpec 生成 → validate() / format_warnings() で受け入れ判定
```

段 5 と段 8 が、この IR に固有の難所である。他の段は一般的な HD マップ自動生成と同じ。

### 2.3 モデルの置き方

| 段 | モデル | 備考 |
| --- | --- | --- |
| 1 位置推定 | LiDAR-inertial オドメトリ（FAST-LIO2 / KISS-ICP 系）+ ポーズグラフ最適化（GTSAM / Ceres）、複数パス間はループ検出 + ICP | **学習不要**。ここの精度が地図全体の上限を決める。RTK 固定解が取れる区間をアンカーにする |
| 3 ベクトル抽出 | 集約 BEV に対するベクトル地図ネットワーク（MapTR / MapTRv2 / PivotNet / StreamMapNet 系、あるいはオフライン集約前提の VMA / GlobalMapNet 系） | オンライン BEV 知覚と違い、**複数パスを集約した後**に 1 回だけ推論するので、時間方向の集約に予算を割ける |
| 4 トポロジー | 車線・車線間、車線・交通要素間の関係推定（TopoNet / TopoMLP 系）＋ 軌跡クラスタリング（横オフセットの DBSCAN / KDE） | 学習モデルと軌跡統計は相補的。実際に走られた movement は軌跡側が確実 |
| 5 当てはめ | **学習ではなく古典最適化**。RDP 簡略化 → 曲率 κ(s) の区分線形フィット + 変化点検出 → 直線 / クロソイド / 円弧への分解 → 非線形最小二乗（Ceres）で端点連続を拘束 | `Alignment` のプリミティブに正確に対応する。κ ≈ 0 なら直線、κ 一定なら円弧、κ 線形ならクロソイド |
| 7 規則推論 | 挙動統計（停止位置分布、減速開始地点、譲り率）＋ ロジスティック回帰程度で十分。標識は検出 + 分類、補助標識の文字は OCR / VLM | 信号現示が観測できるなら停止挙動との相関で制御対象車線を決められる |

VLM / LLM をパイプラインの主役に置く理由はない。使うとしたら標識の補助標識（「7-9時を除く」等）の
読み取りと、`Building.kind` のような語彙の付与くらい。

### 2.4 集めるべき情報

**必須**

| 情報 | レート / 条件 | 何に効くか |
| --- | --- | --- |
| 自車ポーズ（GNSS + IMU + 車輪オドメトリ、共分散付き） | 100 Hz 以上、RTK 推奨 | すべての幾何。`GeoOrigin` / `projection` |
| 時刻同期と外部パラメータ較正 | ハード同期が望ましい | センサ間の整合。ここが緩いと BEV が二重になる |
| **同一区間の複数パス**、できれば全車線を走る | 区間あたり 3 パス以上 | 1 パスでは自車線しか分からない。車線数・対向車線・movement の網羅性はここで決まる |

**準必須**

| 情報 | 何に効くか |
| --- | --- |
| LiDAR（**反射強度付き**） | 白線は再帰反射するので強度 BEV で実線 / 破線が出る。縁石・路肩の 3D 形状、路面平面（= `superelevation`） |
| カメラ（前方 + 可能なら周囲） | マーキングの**色**（白 / 黄）、標識、信号灯器、横断歩道 |

**推奨（あると質が跳ねる）**

| 情報 | 何に効くか |
| --- | --- |
| CAN: 方向指示器 | movement（直進 / 右左折 / 車線変更）のラベルがタダで手に入る。交差点の接続列挙の教師信号 |
| CAN: 速度・加速度・ブレーキ | `RightOfWay` の推論（どちらの流入が譲っているか）、停止線位置、`speed_limit` の代替推定 |
| CAN: 操舵角 | 参照線曲率の事前分布。クロソイド区間の検出と相性がよい |
| 昼夜・天候の多様性 | マーキング可視性のロバスト化 |
| 信号現示（V2I / 灯色認識） | `TrafficRule::TrafficLight` の車線対応付け |

### 2.5 IR の要素ごとの起こし方

| IR 要素 | 必要な証跡 | 手法 | 確度 |
| --- | --- | --- | --- |
| `metadata.origin` / `projection` | GNSS 固定解 | タイル原点を決めて `GeoOrigin`。Autoware 向けなら `Mgrs` | 高 |
| `metadata.handedness` | 走行位置 | 国 / 観測から自明 | 高 |
| `Road.reference_line` | 集約 BEV の境界、または車線中心の平均 | 曲率プロファイルの変化点検出 → line / clothoid / arc の `Composite` | 中 |
| `Road.superelevation` | LiDAR 路面平面の法線 | 平面フィット → ロール角 → `Poly3Profile`。**IMU のロールはサスで汚れる**ので路面から取る | 中 |
| `Road.sections` | 車線数の変化 | 変化点検出。テーパだけなら断面を増やさない（幅プロファイル側） | 中 |
| `Lane.width` | 境界対の横距離 w(s) | ノット化 → `PositiveWidth` 列、`Linear` / `Smooth` は残差比較で選ぶ | 中 |
| `Lane.direction` | 軌跡の向き | 走行済み車線は確実。未走行の対向車線は他パスかマーキング（中央線の黄色）から | 走行済み: 高 / 未走行: 中 |
| `Lane.lane_type` | 画像セマンティクス、縁石高さ | 歩道 / 路肩 / 自転車 / 駐車の分類 | 中 |
| `BoundaryMarking.marking` | 強度 BEV | 実線 / 破線は強度の周期性、二重線は横方向のピーク数、縁石は高さ段差 → `Curbstone` | 中 |
| `BoundaryMarking.color` | カメラ | 露出とホワイトバランスに注意 | 中 |
| `Lane.speed_limit` | 標識認識 | 第一は標識。なければ観測速度の 85 パーセンタイル（**推定値の印を付けること**） | 標識あり: 高 / 推定: 低 |
| `Road.road_type` | 車線数・速度・沿道密度 | 単純な分類器で十分 | 高 |
| `LaneConnection` | 軌跡の遷移 + レーングラフ | 観測された遷移は確実。未観測の合法 movement は幾何から補完 | 観測: 高 / 補完: 中 |
| `Junction` | マーキング消失 + 分岐集中 | 区画を検出してアームを停止線位置まで切り戻す。コネクタは `connect_via` に任せる | 低〜中（**最難**） |
| `MapObject::StopLine` | 強度 BEV の横線 + 停止イベント分布 | 両者の一致で確度が上がる | 高 |
| `MapObject::Crosswalk` | ゼブラ検出 | `Band { left, right }` に落とす | 高 |
| `MapObject::TrafficLight` | 複数視点の三角測量 | 複数パスで平均。高さは路面からの相対で持つ | 中 |
| `MapObject::TrafficSign { code }` | 標識分類器 | カタログは IR が定義しないので、日本の標識番号をそのまま通せる | 中 |
| `TrafficRule::TrafficLight` | 灯器の向き + 流入車線の幾何、現示と停止挙動の相関 | 灯器 1 基が複数車線を支配するので、対応付けは 1 対多 | 低〜中 |
| `TrafficRule::RightOfWay` | **譲り挙動の統計** | 競合点でどちらが減速 / 停止するかの頻度。これは測量では取れず、走行ログでしか取れない情報 | 低〜中だが価値は最大 |
| `TrafficRule::SpeedLimit` | 上に同じ | 車線単位の制限（例: 登坂車線）は観測分離が難しい | 低 |
| `buildings` | 地上 LiDAR のファサード | 地上からは**壁しか見えない**。輪郭は OSM フットプリント、高さは実測、屋根形状は航空データがないと当たらない | 低（任意） |

### 2.6 ログからは起こらないもの

- **屋根形状と建物用途**。`RoofShape` は「消費者に伝わる語」に限定されているが、
  地上走行ログにその語を決める証拠がない。`Roof::FLAT` の押し出しに留めるのが誠実で、
  IR もそれを許している（分からない屋根は近い形に寄せず、含む体積の平らな上面になる、という設計）。
- **未走行・未観測の車線の細部**。IR は部分観測を表現しない。`Road` は必ず車線を持ち、
  車線は必ず幅を持つ。「見えなかった」は書けないので、埋めるか、その道路を出さないかの二択になる。
- **不確かさと出典**。IR は「確定した地図」だけを表現し、共分散も観測回数も観測日も置く場所がない。
  これは設計の一貫性（形式が表現できないものを IR に押し込まない、という方針）と同じ理由で、
  IR に足すのではなく **サイドカー（IR の ID をキーにした別ファイル）** に持つのが筋が良い。
  ID が入力から導かれる安定なものなので、これは実際にやりやすい。
- **時間変化**。工事規制、時間帯規制、可変速度。IR に時間軸はない。

### 2.7 最小構成から始めるなら

1. **単路のみ、単一パス、方向は 1 つ**。RTK ポーズだけで参照線を起こし、
   車線は固定幅 1 本、マーキングは `Solid`。`export_opendrive` が通ることを確認する。
   ここまでは知覚モデルなしで到達できる。
2. **強度 BEV から境界を出す**。車線数と幅プロファイルが入り、`CrossSection` が意味を持ち始める。
3. **複数パスの集約**。対向車線と全車線被覆。`LaneConnection` が軌跡から埋まる。
4. **交差点**。アームの切り戻しと movement 列挙。ここが本番。
5. **物体と規則**。停止線 → 横断歩道 → 信号 → 優先関係の順に、確度の高いものから。

受け入れテストは既にリポジトリ側にある。`validate()` の `issues()` が空であること、
各エクスポータの `check()`（Python では `format_warnings()`）が静かであること、
そして README の「OpenDRIVE と Lanelet2 の一致」テストと同じ比較を、
起こした地図に対して回すこと。

### 2.8 この IR に逆生成を足すときの注意

- **`Map` を直接組まず、`MapBuilder` を通す。** 直接組むと境界共有・断面整合・
  コネクタ生成を自前で正しくやる必要があり、1 mm の許容差がそれを罰する。
- **識別子の決め方を先に決める。** カウンタは使わない設計なので、
  タイル ID + 幾何ハッシュ、または元になった OSM way ID のような
  「入力から導かれる安定な鍵」を最初に決めておくと、ログが増えて再生成したときに
  地図の diff が読める。これは地図メンテナンスの実務では効く。
- **`Curve3::Polyline` を最終形にしない。** 暫定としては通るが、そのままでは
  ノイズが OpenDRIVE の幾何に出る。段 5 の当てはめまでやって初めて使い物になる。
- **`superelevation` は検証で妥当性を見られている**（`ImplausibleSuperelevation`）。
  車体ロールから起こすと簡単に引っかかる。

---

## 付録: 参照した場所

| 内容 | ファイル |
| --- | --- |
| IR 本体（`Map` / `Road` / `Lane` / `CrossSection`） | `crates/roadgen-core/src/map.rs` |
| トポロジー | `crates/roadgen-core/src/topology.rs` |
| セマンティクス | `crates/roadgen-core/src/semantics.rs` |
| 曲線 / プロファイル / 幅 | `crates/roadgen-core/src/geometry/` |
| 型で潰した制約 | `crates/roadgen-core/src/units.rs` |
| 識別子 | `crates/roadgen-core/src/id.rs` |
| 生成器の入力（逆生成のターゲット） | `crates/roadgen-core/src/builder.rs` |
| 検証ゲートと許容差 | `crates/roadgen-core/src/validation.rs` |
| 建物 | `crates/roadgen-core/src/buildings.rs` |
| Python API | `crates/roadgen-python/src/lib.rs` |
