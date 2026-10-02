# IR ダンプとフォーマット間 ID トレース — 設計案

*2026-09 設計（改訂 2: エクスポータごとのトレースファイル方式）。フェーズ 1 実装済み。*

## 0. 実装状況（フェーズ 1）

| 部品 | 場所 |
| --- | --- |
| `IrRef` / `Relation` / `TraceLink` / `Trace` | `crates/roadgen-core/src/trace.rs` |
| IR ダンプ、トレースファイル、`TraceIndex`、`.net.xml` 読込 | `crates/roadgen-trace` |
| OpenDRIVE | `roadgen_opendrive::{trace, to_opendrive_traced, to_xml_traced, write_traced, write_with_traced}` |
| Lanelet2 | `roadgen_lanelet2::{to_lanelet_map_traced, to_osm_xml_traced, write_traced}` |
| OSM | `roadgen_osm::{to_xml_traced, write_traced}` |
| SUMO | `PlainNetwork.trace`、`roadgen_sumo::write_traced` |
| ClipGT | `roadgen_clipgt::{to_layers_traced, write_traced}` |
| GPUDrive | `roadgen_gpudrive::{to_scene_traced, to_json_traced, write_traced}` |
| CARLA | `roadgen_carla::write_traced`（xodr と信号・標識の actor のみ） |
| Python | 全 `export_*` が既定でトレースファイルを書く（`trace=False` で抑止）、`Map.export_ir`、`roadgen.Trace` |

設計からの変更点:

- ディレクトリ出力のトレースファイル名は `<dir>/<名前>.<format>.trace.json`。名前は SUMO の prefix、ClipGT の clip id、CARLA の地図名で、同じディレクトリに複数のフォーマットや複数のネットワーク・clip を書いても衝突しない。
- `options` フィールドは入れていない。採番に効くオプションは OpenDRIVE の信号配置だけで、その影響はトレース自体に反映される（書かれなかった要素にはリンクが無い）。
- `translate` の近傍への拡張は、元要素のどの IR 要素にも対象フォーマットの対応が無いときだけ行う。一部でも直接の対応があればそれだけを返す（SUMO の内部 lane → OpenDRIVE で、connector lane の答えに周辺の lane が混ざらないように）。
- IR ダンプは形状を載せないが、fingerprint は形状も含めて計算する。カタログの `geometry` 欄に IR 全体（形状・sampling・建物の形など）の digest を入れ、それも fingerprint の対象にした。ID とつながりが同じでも形状の違う地図（例: 停止線を動かすと OSM でのノード統合が変わる）は別の地図として扱われ、トレースが突き合わされない。
- IR ダンプの読込時には中身から fingerprint を再計算し、埋め込まれた値と違えば拒否する。
- GPUDrive の agent は、軌跡が実際に届いた lane（role `route`）と、届かなかった場合のゴールの lane（role `goal`）にだけ結び付ける。
- 読み込み側（`read_opendrive`）のトレース、CLI、Web デモのハイライトはフェーズ 2 のまま。

## 1. 何を解きたいか

roadgen は 1 つの Canonical Road IR から 7 つのフォーマットを書く。書かれた側の ID は
それぞれ別の規則で振られ、IR 側には何も戻らない（`id.rs` の doc: "OpenDRIVE ids and
Lanelet2 ids are assigned by the exporters, from these, and never flow back"）。
その結果、次のような問いに今は答えられない:

- Autoware で問題が出た lanelet `1000123` は、OpenDRIVE のどの road / lane か？
- SUMO の `north.fwd_1` は Lanelet2 のどの lanelet か？
- 外部から読んだ `.xodr` の road 12 / lane -2 は、書き出した Lanelet2 のどれになったか？

実際 `tests/integration/tests/cross_format.rs` は IR→OpenDRIVE の対応
（road 番号 = arena 順、lane 番号 = ±ordinal）を**テスト側で手書きで再導出**している。
この知識をライブラリの出力にしたい、というのが本提案。

## 2. 重要な前提: IR ダンプ単体ではトレースできない

調査結果（各 exporter の ID 採番規則）:

| フォーマット | 出力 ID の決まり方 | IR から純関数で再計算できるか | 既存の対応表 |
| --- | --- | --- | --- |
| OpenDRIVE | road = arena 順の番号、junction = roads.len()+順、lane = ±ordinal（section 内）、signal/object = objects 順、building = objects.len()+順 | ○（`Numbering`） | private `Numbering`、public な `lane_id`/`junction_id`/`signal_id` |
| SUMO | 文字列。edge = `<road>[.<section>].<fwd\|bwd>`、lane = `<edge>_<右から数えた index>`、node = `j_<junction>` 等 | ○（ただしオプションとセクション構成に依存） | public `PlainNetwork.lanes` のみ |
| Lanelet2 | **連番**（points 1000〜、その他 1001000〜、構築順） | ✕（構築過程をなぞる必要がある） | private `Exporter.lanelets` / `objects` / `boundaries`、`run()` で捨てられる |
| OSM | **負の連番**（ノード・ウェイ・リレーション共通） | ✕ | private `ways` / `junction_nodes` |
| CARLA | `.xodr` は OpenDRIVE と同一。メッシュ名は `<map>_<kind>_<subtype>_<ordinal>` | 部分的 | furniture manifest（object ↔ signal ↔ actor）は既に JSON 出力 |
| ClipGT | Parquet の**行番号**のみ | ✕ | なし |
| GPUDrive | 配列 index のみ | ✕ | なし |

→ Lanelet2 / OSM / ClipGT / GPUDrive の ID は IR に含まれない情報（構築順・フィルタ・
重複排除）で決まる。**IR をダンプしただけでは対応は復元できない**。
トレースは「IR ダンプ」＋「各 exporter が書き出し時に記録した対応表」の組で初めて成立する。

## 3. 全体像: IR ID をハブにしたスター型

```
   lanelet2:lanelet:1000123 ─┐                     ┌─ opendrive:lane:0/1/-1
   sumo:lane:north.fwd_0 ────┼──▶  lane/north/0  ──┼─ clipgt:lane:17
   osm:way:-42 ──────────────┘   (IR の安定 ID)     └─ gpudrive:road:5
```

フォーマット間を直接つなぐ表は作らない。各フォーマットは **IR ID との対応だけ**を持ち、
A→B の変換は `A → IR ID(s) → B` の合成で行う。フォーマットが 7 個あっても表は 7 本で済み、
新しい exporter を足しても既存の表に触らない。

ハブになれるのは IR ID が「呼び出し側の入力から導かれ、カウンタではない」からで、
これは既存設計（`id.rs`）がすでに保証している性質。

## 4. 成果物: IR ダンプ 1 本 + エクスポータごとのトレースファイル

各エクスポータが自分の出力の横にトレースファイルを書き出す。IR は別に 1 本ダンプする。
追跡するときはこれらを IR ID で突き合わせる。

```
out/
├── map.ir.json                        ← IR ダンプ（export_ir）
├── map.xodr
├── map.xodr.trace.json                ← OpenDRIVE エクスポータが書く
├── lanelet2_map.osm
├── lanelet2_map.osm.trace.json        ← Lanelet2 エクスポータが書く
└── sumo/
    ├── net.nod.xml …
    └── town.sumo.trace.json           ← SUMO エクスポータが書く（ディレクトリ出力は中に `<名前>.<format>.trace.json`）
```

例: Lanelet2 の lanelet `1000123` → SUMO

```
lanelet2_map.osm.trace.json で逆引き   lanelet:1000123 → lane/north/0
sumo/town.sumo.trace.json で正引き               lane/north/0    → lane:north.fwd_0
```

1 ファイルにまとめる案（旧版）ではなくこの形にする理由:

- **エクスポータ同士が独立したままになる。** 各エクスポータは自分の対応表を書くだけで、
  PyMap 側に「ここまでの export を溜めておく」状態を持たなくてよい
- **別の時点・別プロセスで書き出したファイル同士も突き合わせられる。**
  同じ IR から出たものかは `ir_fingerprint` で判定する
- **書き出したフォーマットの分だけファイルができる。** 使わないフォーマットの trace は存在しない
- エクスポータを 1 つ足しても、既存のトレースファイルにも IR ダンプにも影響しない

### 4.0 IR ダンプとトレースファイルの役割分担

- 両方のフォーマットに対応物がある要素（lane ↔ lanelet ↔ SUMO lane など）なら、
  **トレースファイル 2 本を IR ID で join するだけで追える。** IR ダンプは使わない
- IR ダンプが必要になるのは次の 3 つ:
  1. **片方にしかない要素**。例えば交差点内の lane は SUMO に存在しない。
     IR の関係（connector lane → それを通る connection、lane → road）をたどって、
     近い対応物に広げる
  2. **正しい組み合わせかの検証**。`fingerprint` を持ち、各トレースファイルの
     `ir_fingerprint` と照合する
  3. **IR ID の意味を人が読む**。どの road のどの section の何番目の lane か

**`map.ir.json`**

```jsonc
{
  "schema": "roadgen-ir/1",
  "generator": { "name": "roadgen", "version": "0.3.2" },
  "fingerprint": "sha256:…",           // 以下の本体を正規化した JSON のハッシュ
  "metadata": { "name": "town", "handedness": "right", "projection": "…", "origin": {…} },
  "roads":       [{ "id": "road/north", "name": "north", "junction": null,
                    "sections": [{ "station": 0.0, "lanes": ["lane/north/0", "lane/north/1"] }],
                    "predecessor": null, "successor": {"junction": "junction/j0"} }],
  "lanes":       [{ "id": "lane/north/0", "road": "road/north", "section": 0,
                    "side": "right", "ordinal": 1, "direction": "forward", "type": "driving" }],
  "junctions":   [{ "id": "junction/j0", "incoming": [...], "connecting": [...] }],
  "connections": [{ "id": "connection/j0/north_0/east_0",
                    "from": "lane/north/0", "to": "lane/j0_c3/0", "junction": "junction/j0" }],
  "objects":     [{ "id": "object/tl_0", "kind": "traffic_light", "lanes": ["lane/north/0"] }],
  "rules":       [{ "kind": "right_of_way", "lanes": [...] }],
  "buildings":   [{ "id": "building/north/left/3/0", "parts": ["part/north/left/3/0/0"] }]
}
```

**`<出力>.trace.json`**（全フォーマットで同じスキーマ）

```jsonc
{
  "schema": "roadgen-trace/1",
  "generator": { "name": "roadgen", "version": "0.3.2" },
  "format": "lanelet2",
  "direction": "export",                 // "export"（IR→ファイル）か "import"（ファイル→IR、§5.3）
  "ir_fingerprint": "sha256:…",          // どの IR ダンプと組になるか
  "files":   [{ "path": "lanelet2_map.osm", "sha256": "…" }],
  "options": { … },                      // 採番に影響するオプションだけ
  "links": [
    { "ir": "lane/north/0", "ref": "lanelet:1000123",    "rel": "exact",  "role": "lanelet" },
    { "ir": "lane/north/0", "ref": "linestring:1000124", "rel": "exact",  "role": "centerline" },
    { "ir": "lane/north/0", "ref": "linestring:1000010", "rel": "merged", "role": "left_boundary" },
    { "ir": "object/tl_0",  "ref": "regulatory_element:1000500", "rel": "exact" }
  ]
}
```

OpenDRIVE なら `{"ir": "lane/north/0", "ref": "lane:0/0/-1"}`、
SUMO なら `{"ir": "road/north", "ref": "edge:north.fwd", "rel": "part"}` のように、
`links` の形は全フォーマットで共通。

### 4.1 IR ダンプの範囲（フェーズ 1 は「カタログ」）

トレースに要るのは **ID・所属・トポロジ・意味属性**で、幾何は要らない。
フェーズ 1 のダンプはこの「IR カタログ」に絞る:

- 小さい（数千 lane でも数百 KB）、diff が読める、スキーマが安定しやすい
- 再生成前後の `map.ir.json` を diff すれば「ID が変わった / 消えた」が一目で分かる（副次効果）

幾何まで含む完全ダンプ（再読込で `Map` を復元できるもの）はフェーズ 3 に回す（§8）。

### 4.2 `ref` = フォーマット内ローカル参照

`"<kind>:<local>"` の文字列。フォーマット名は外側のキー。
グローバルに書くときは `"<format>:<kind>:<local>"`（例 `lanelet2:lanelet:1000123`）。

| format | kind 例 | local の形 |
| --- | --- | --- |
| opendrive | `road` `junction` `lane` `signal` `object` `building` `connection` `controller` | `lane:<road>/<section index>/<lane no>`、`connection:<junction>/<connection id>` |
| lanelet2 | `lanelet` `linestring` `point` `regulatory_element` | 整数 |
| sumo | `edge` `lane` `node` `connection` `crossing` `tls` | SUMO の ID 文字列、connection は `<fromLane>><toLane>`、crossing は `<node>/<edge>+<edge>`（edge は名前順）、tls は `<tlLogic id>`、そのスロットは `<tlLogic id>/<linkIndex>`（§8.2） |
| osm | `node` `way` `relation` | 負の整数 |
| carla | `mesh` `actor` + `.xodr` 分は opendrive を参照 | 名前 |
| clipgt | レイヤ名 (`lane` `lane_line` `wait_line` …) | 行番号 |
| gpudrive | `road` `agent` | 配列 index |

OpenDRIVE の lane 番号は section 内でしか一意でないので、section index を必ず含める。

### 4.3 `rel` = 多重度

IR とフォーマットは 1:1 とは限らない。これを明示しないと逆引きが嘘をつく。

| rel | 意味 | 例 |
| --- | --- | --- |
| `exact` | 1 IR 要素 ↔ 1 出力要素 | lane ↔ lanelet、lane ↔ SUMO lane |
| `part` | 1 IR 要素が複数出力に分割され、この出力はその一部 | road → SUMO edge（方向×section）、building → OSM part way |
| `merged` | 複数 IR 要素が 1 出力に統合され、この IR はその一部 | 共有境界 linestring、OpenDRIVE `<connection>`、OSM ノードに乗った信号 |
| `collapsed` | 出力に対応物がなく、別の要素に吸収された | SUMO では connector road の lane → そのまま `<connection>` に畳まれる |

`role` は同じ IR 要素から複数出力が出るときの区別（`lanelet` / `centerline` /
`left_boundary` …）。出力に現れない IR 要素（ClipGT の非走行 lane 等）は
**リンクを書かない**＝「このフォーマットには存在しない」を意味する。

### 4.4 整合性チェック

Lanelet2 / OSM / ClipGT の ID は構築順で決まるので、ファイルを作り直すと古い trace は
**黙って間違った答え**を返す。これを防ぐため:

- `files[].sha256` — 読込時に実ファイルと照合し、不一致なら警告/エラー
- `ir_fingerprint` — トレースファイル同士、およびトレースファイルと `map.ir.json` が
  同じ IR から出たものかの判定。ファイルが分かれる分、これが突き合わせの前提になる。
  一致しない組み合わせは `TraceIndex` が読込時に拒否する
- `generator.version` — 採番規則の変更を追うため

## 5. Rust 側の設計

### 5.1 `roadgen-core::trace`（フォーマット非依存）

core はフォーマットを知らないという既存の分離を守り、型だけを置く:

```rust
pub enum IrRef { Road(RoadId), Lane(LaneId), Junction(JunctionId),
                 Connection(ConnectionId), Object(ObjectId),
                 Building(BuildingId), BuildingPart(BuildingPartId), Rule(usize) }

pub enum Relation { Exact, Part, Merged, Collapsed }

pub struct TraceLink { pub ir: IrRef, pub local: String /* "<kind>:<local>" */,
                       pub relation: Relation, pub role: Option<&'static str> }

#[derive(Default)]
pub struct Trace { pub format: &'static str, pub links: Vec<TraceLink> }
```

`IrRef` は「任意の IR 要素を 1 つの型で指す」もので、`TrafficRule` には ID が無いので
`rules` の index で指す（ルールに ID を振るのは別途検討）。

### 5.2 各 exporter: `*_traced` を追加（既存 API は不変）

| crate | 追加 API | 実装方針 | 手間 |
| --- | --- | --- | --- |
| roadgen-opendrive | `trace(&map, &options) -> Trace` | `Numbering` をそのまま Trace に変換。純関数なので書き出し不要。junction `<connection>` / controller は `junction()` / `signal_groups` の採番を関数に切り出して共有 | 小 |
| roadgen-sumo | `PlainNetwork.trace` | 既存 `lanes` に加え、`edge_id()` / node 命名 / connection の BTreeSet から road→edge、junction→node、connection を記録 | 小 |
| roadgen-lanelet2 | `export_traced() -> (Arc<LaneletMap>, Trace)` | `Exporter::run` の最後に `lanelets` / `objects` / `boundaries` / `own_boundaries` を Trace に畳む。reg-elem は `build_rules` で記録を追加 | 小〜中 |
| roadgen-osm | 同上 | `ways` / `junction_nodes` を流用。furniture のノード統合・building way・restriction relation は記録を追加 | 中 |
| roadgen-clipgt | 各レイヤの行を push する箇所で `(IrRef, layer, row)` を記録 | 行番号 = ID なので push 時に記録するのが唯一の方法 | 中 |
| roadgen-gpudrive | 同上（`roads.len()` を採番している箇所） | | 中 |
| roadgen-carla | `.xodr` 分は opendrive の Trace を再利用。furniture manifest は既存。メッシュは `Mesh` に `sources: Vec<IrRef>` を持たせる | メッシュ対応が最大の手間。フェーズ 2 に回す | 大 |

**鉄則: 採番ロジックを二重に書かない。** trace は exporter が実際に使った表から作る。
OpenDRIVE の `signal_id` 等のように「別途 `Numbering::new` を呼び直す」形は、
採番と trace がずれる余地があるので、trace は同じインスタンスから出す。

### 5.3 リーダ側: `read_opendrive` にも Trace を

外部 `.xodr` → IR → Lanelet2 のトレースが本機能で一番価値のあるユースケースになる
（「元データのこの lane が Autoware のどの lanelet か」）。現状:

- road / junction は `road/<xodr id>` として IR ID に残る → そのまま追える
- **lane 番号は失われる**（`Reader.lanes: HashMap<(RoadId, section, i64), LaneId>` が
  `run()` 終了時に捨てられる）
- object は `name` が `object/…` ならそれ、なければ `object/<xodr id>`（重複時 `-N`）

→ `Imported` に `trace: Trace`（format = `"opendrive"`）を追加し、`Reader.lanes` と
`signal_ids` / building 名をそこへ出す。ファイルとしては `direction: "import"` の
トレースファイル（例 `input.xodr.trace.json`）になり、スキーマはエクスポート側と同じ。
これで「入力 xodr の lane → 出力 lanelet」も、入力側のトレースファイルと
Lanelet2 のトレースファイルの join で引ける。

### 5.4 検索 API: `TraceIndex`

```rust
let mut index = TraceIndex::new();                        // fingerprint / sha256 照合は load 時
index.load("out/map.ir.json")?;                           // IR ダンプ（任意だが推奨）
index.load("out/lanelet2_map.osm.trace.json")?;
index.load("out/sumo/town.sumo.trace.json")?;
index.load_sumo_net("out/sumo/town.net.xml")?;            // 任意: 交差点内 lane（§8.1）
index.to_ir("lanelet2", "lanelet:1000123")                 // -> Vec<&Link>
index.from_ir("lane/north/0", "sumo")                      // -> Vec<&Link>
index.translate("lanelet2", "1000123", "sumo")             // 合成（種別は省略可）
```

`translate` の合成規則:

1. `from` の ref → IR 要素集合（`merged` なら複数）
2. 対象フォーマットにリンクがない IR 要素は、IR ダンプの関係で**1 段だけ**広げる
   （lane → その road、connector lane → それを通る connection）。広げた場合は
   結果に `via` を付けて「近似」であることを返す
3. 各 IR 要素 → `to` の ref 集合

結果は常に集合で返し、1:1 を仮定しない。IR ダンプを読んでいない場合は 2 を飛ばし、
対応物がなければ空集合を返す（トレースファイルだけでも join はできる）。

## 6. Python / CLI

トレースファイルはエクスポート時に出力の横へ書く。既存の呼び出しはそのまま動き、
`trace=False` で書かないこともできる:

```python
m.export_opendrive("out/map.xodr")         # + out/map.xodr.trace.json
m.export_lanelet2("out/lanelet2_map.osm")  # + out/lanelet2_map.osm.trace.json
m.export_sumo("out/sumo/")                 # + out/sumo/town.sumo.trace.json
m.export_ir("out/map.ir.json")             # IR ダンプ

t = roadgen.Trace.load("out/map.ir.json",
                       "out/lanelet2_map.osm.trace.json",
                       "out/sumo/town.sumo.trace.json")
t.to_ir("lanelet2", 1000123)                       # ['lane/north/0']
t.translate("lanelet2", 1000123, to="sumo")        # [('lane', 'north.fwd_0')]
t.translate("sumo", "north.fwd_0", to="lanelet2")
```

```
python -m roadgen trace out/map.ir.json out/*.trace.json out/sumo/town.sumo.trace.json \
    lanelet2:lanelet:1000123 --to sumo
```

トレースファイルは既定で書く（決定済み）。出力ディレクトリにファイルが 1 本増えるが、
後から同じ IR を再現できない限りトレースは作り直せないため。

Web デモ（viewer）では、同じ Trace を使って「あるフォーマットで要素をクリック →
他フォーマットの対応要素をハイライト」ができる。フェーズ 2 以降の候補。

## 7. テスト

- `cross_format.rs` の手書き対応（`opendrive_lane_id`、「n 番目の road は n 番目」）を
  `trace` に置き換え、**trace 自体を既存の幾何比較で検証する**:
  trace が lane A ↔ lanelet X と言うなら、両者の中心線が一致すること
- 各 exporter: 出力ファイルを再パースし、trace の全 `ref` が実在すること、
  `exact` の IR 要素が重複しないこと
- `reimport.rs`: xodr → IR → xodr で 入力側（import）と出力側（export）のトレースファイルを join すると
  元の lane 番号に戻ること
- 決定性: 同じ入力から 2 回作った trace が byte 単位で一致すること

## 8. 対象範囲と段階的導入

**対象**: 3D モデル（CARLA の FBX メッシュ）と点群地図**以外の全フォーマット**。
CARLA パッケージ内の `.xodr` は OpenDRIVE のトレースで、信号・標識は既存の
furniture manifest（IR object ↔ xodr signal ↔ actor 名）で追える。

| フォーマット | ref の単位 | 注意点 |
| --- | --- | --- |
| OpenDRIVE | road / section 内 lane / junction / connection / signal / object / building / controller | `Numbering` を流用。純関数 |
| Lanelet2 | lanelet / linestring / regulatory_element | 連番。exporter の private 表を返すだけ |
| OSM | node / way / relation | 信号等はノードに統合（`merged`）、restriction は腕のペア単位で複数 connection に対応 |
| SUMO | node / edge / lane / connection / tls | plain XML の ID を記録。信号は `tls:<id>` とスロット `tls:<id>/<linkIndex>`（§8.2）。netconvert が作る内部 lane（`:j_x_0_0`）は、`.net.xml` の `<connection from to fromLane toLane via>` を読んで connection 経由で解決する（§8.1） |
| ClipGT | `<layer>:<row>` | 行番号が唯一の ID。走行不可 lane は出力されない（リンク無し）、lane_line は重複排除で `merged` |
| GPUDrive | `road:<index>` / `agent:<id>` | 行番号。edge/line は重複排除で `merged`。agent は走行ルートの lane 列に `part` で対応 |
| CARLA（xodr・furniture のみ） | opendrive の ref + actor 名 | FBX メッシュは対象外 |

### 8.1 SUMO の内部 lane

内部 lane（`:j_x_0_0`）は netconvert が作るもので、roadgen が書く plain XML には無い。
`netconvert --output.original-names true` を試したが（SUMO 1.18、crossroads シナリオ）、
`origId` は 1 つも出力されなかった。plain XML には元の名前という概念が無いため、
このオプションは効かない。

一方、`.net.xml` の `<connection>` は次の形で内部 lane を名指ししている:

```xml
<connection from="north.fwd" to="west.bwd" fromLane="0" toLane="0" via=":j_x_0_0" .../>
```

`from` / `to` / `fromLane` / `toLane` は roadgen が付けた ID そのままなので、
トレースファイルに記録した connection（`connection:north.fwd_0>west.bwd_0` → IR の
connection）と突き合わせれば `via` の内部 lane も IR まで追える。
`TraceIndex` が `.net.xml` を任意で受け取り、その場でこの対応を足す。netconvert の
オプションには依存しない。

netconvert が独自に足す接続（行き止まりの U ターン `north.bwd → north.fwd` など）は
IR に対応物が無いので、リンクなしとして扱う。

**追記（実装後）**: U ターンは SUMO だけが持つ動き（他のフォーマットでは lane がそこで終わる）なので、
エクスポートする `.netccfg` に `no-turnarounds` を入れて netconvert に作らせないことにした。
この設定で作ったネットワークでは、via 付きの接続はすべてトレースできる（crossroads で 14 本中 14 本）。
設定を使わずに作った場合は、従来どおり U ターンがリンクなしになる。

注意点が 1 つある。左折のように交差点内で一時停止する動きでは、netconvert が内部 lane を
**2 本に分ける**（`:j_x_2_0` の後に `:j_x_12_0`）。後半は内部 edge 起点の接続
`<connection from=":j_x_2" to="east.bwd" ... via=":j_x_12_0"/>` として現れるので、
`via` の連鎖をたどり、どちらも同じ IR connector lane に `rel: part` で対応させる。

**検証結果**（SUMO 1.18、公開 API だけで書いた検証用プログラム。コミットはしていない）:

| シナリオ | `.net.xml` の接続（`via` 付き） | IR connector lane まで解決 | IR に対応なし |
| --- | --- | --- | --- |
| crossroads | 16 | 12（全 movement。例: `:j_x_0_0` → `lane/x/north_0_west_1/0` → OpenDRIVE road 8 / lane -1） | 4（行き止まりの U ターン） |
| controlled_crossroads | 4 | 2 | 2（同上） |

解決の手順: `via` の接続の `from`+`fromLane` と `to`+`toLane` を SUMO のトレースで
IR lane に戻し、IR 上で「進入 lane → connector lane → 退出 lane」となる connector lane を
探す。候補はどのケースでも 1 本に決まった。

### 8.2 SUMO の信号

netconvert に信号のプログラムを作らせると、フェーズも connection の linkIndex の振り方も
netconvert のものになり、バージョンやジャンクションの形状の読み方で変わる。これでは
IR の信号機がどのプログラムのどのスロットを制御するかをトレースに書けない。

そこでエクスポータがプログラムを自分で決めて書く（`roadgen_sumo::signals`）:

- 信号付きノードには IR の junction から決まる名前を付ける（ノード id と同じ。`<node tl="j_x">`）
- `<prefix>.tll.xml` に `<tlLogic id="j_x" programID="0" type="static" offset="0">` のフェーズと、
  ノードに入る全 connection の `<connection ... tl="j_x" linkIndex="n"/>` を書き、`.netccfg` の
  `tllogic-files` で読ませる。plain の `.con.xml` のスキーマは `tl` / `linkIndex` を許さないので、
  これらは `.tll.xml` 側に置く（SUMO 1.26 で確認）
- フェーズは Autoware `lanelet2_to_sumo` の静的ヒューリスティックに倣う。グループ分けは
  「向かい合う進入路」だけをまとめる: 進入路は、既存グループの**全メンバー**と進入方位が
  180° ± 35° 以内（＝同じ軸を逆向きに走って交差点で向かい合う）のとき、そのうち最初の
  グループに入り、どれにも入れなければ新しいグループを作る。軸 mod 180° が近いだけでは
  まとめない — 鋭角に交わる 2 本の道路の隣り合う腕は軸が 35° 以内でも向かい合っておらず、
  直進同士が交差するため別グループにする。「全メンバー」とするのは、少しずつ傾いた腕が
  連鎖して交差点全体が 1 フェーズになるのを防ぐため。グループごとに青 35 s（3 グループ以上なら
  25 s）・黄 3 s・全赤 2 s。青の中で、対向を横切る動き（右側通行の左折、左側通行の右折、
  そして ±150° を超える U ターンは符号によらず）でそのフェーズに対向進入路が実際にある
  ものと、同じ出口 lane に入る他の動きがある動きは `g`、それ以外は `G`。対向を横切るかは
  netconvert の `dir` ではなく、lane の幾何と `metadata.handedness` から判定する
- 信号付きノードに入る edge で、IR が先への動きを 1 本も書いていないもの（歩道のみの edge を除く）は、
  `.con.xml` にノードから出る全 edge への `<delete from to/>` を書く。書かないと netconvert が
  動きを推測し、その connection はどのプログラムにも属さない（`tl` 無し、状態 `m`）＝信号を
  無視して交差点に入る車両になる。これでノードに入る非内部 connection はすべて `.tll.xml` の
  `tl` / `linkIndex` を持つ（SUMO 1.18 / 1.26 で確認）

トレースは次のとおり:

| IR | ref | rel | role |
| --- | --- | --- | --- |
| 信号機（object） | `tls:<id>` | merged | `traffic_light` |
| 信号機（object） | `tls:<id>/<linkIndex>`（その信号機が制御する lane から出る各 connection） | merged | `link` |
| `TrafficLight` ルール | `tls:<id>` | merged | `traffic_light` |

スロットがどの connection かは、`.tll.xml` の `<connection>`（または `.net.xml` の
`<connection tl linkIndex>`）で引く。`TraceIndex::load_sumo_net` は、トレースの `tls:` がすべて
`.net.xml` にあること、および `.tll.xml` に書いた各 connection の `tl` / `linkIndex` が `.net.xml`
と一致することを検査し、一致しないネットワーク（例えば netconvert が自分でプログラムを作ったもの）は拒否する。

横断歩道（`<crossing>`）のある信号付きノードでは、netconvert は `.tll.xml` のプログラムを受け入れたうえで
拡張する: 車両の linkIndex 0..n-1 はそのまま、横断歩道の link を n 以降に追加し、各青を
分割して末尾に歩行者クリアランスを入れる（例: 35 s → 30 s + 5 s。黄・全赤はそのまま。
SUMO 1.18 / 1.26 で確認、警告なし）。追加されたスロットは netconvert のものなので
`load_sumo_net` は検査もリンクもせず、エクスポータが書いた車両スロットだけを検査する。

制御する connection が 1 本も無い信号付きノードはプログラムを書かず、従来どおり信号機を
`node:<id>` に `merged` で対応させる。

| フェーズ | 内容 |
| --- | --- |
| 1 | `roadgen-core::trace` 型、IR ダンプ（カタログ）、上表の全フォーマットのトレースファイル（既定で書き出し、`trace=False` で抑止）、`TraceIndex`、Python `export_ir` / `Trace.load` / `translate`。core には serde を入れず、JSON 化は専用の小 crate（`roadgen-trace`）に置く |
| 2 | `read_opendrive` の読み込み側トレース、CLI、Web デモでの対応要素ハイライト |
| 3 | 完全 IR ダンプ（幾何込み）。`roadgen-core` に `serde` feature を足し、`Curve3` / `WidthProfile` 等まで derive、`Map` を JSON から復元 → `validate()`。スキーマ維持コストが大きいので需要を見てから |

## 9. 決めてほしいこと

0. ~~トレースファイルを既定で書き出すか~~ → **決定: 書き出す**（`trace=False` で抑止）。

1. **ダンプの範囲**: フェーズ 1 は「ID/トポロジ/属性のカタログ」で良いか、最初から
   幾何込みの完全ダンプ（再読込可能）が欲しいか。
2. **ファイル形式**: JSON で良いか（大規模マップで重ければ `links` だけ
   Parquet/CSV に逃がす選択肢もある）。
3. ~~優先フォーマット~~ → **決定: 3D モデル・点群地図以外の全フォーマット**。
   残る論点は外部 xodr 起点の追跡（§5.3）をフェーズ 1 に入れるか。
4. **出力への ID 埋め込み**: 補助として、Lanelet2 の lanelet に `roadgen:id=lane/north/0`
   タグを付ける等、ファイル自体に IR ID を書く案もある（trace ファイルが無くても
   追える）。Autoware の読み込みに影響しない範囲で併用するか。
