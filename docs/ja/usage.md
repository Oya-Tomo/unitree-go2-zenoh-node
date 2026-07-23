# 使い方

[セットアップ](setup.md)と実機の安全確認を完了してから、この手順を使ってください。

## 起動と終了

robot nodeを起動するshellでCycloneDDS prefixを設定します。

```console
$ export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
$ uv run node.py
```

nodeは有効な`SportModeState`を受信するまでcommandを受け付けません。
nodeがStateを受信してから、別terminalでkeyboardを起動します。

```console
$ uv run examples/keyboard.py
```

Shiftを離したとき、windowがfocusを失ったとき、windowを閉じたとき、またはprogramを終了したとき、keyboardは速度0を送信します。
robot nodeより先にkeyboardを終了してください。

`Ctrl-C`または`SIGTERM`を受けると、robot nodeはDDSを閉じる前にState-drivenなDown workflowを実行します。
無線remoteは常に使える状態にしてください。
通常のprocess終了は緊急停止ではなく、通信またはSDKが利用できない場合に動作停止を保証できません。

## CLI option

### Robot node

| Option | 既定値 | 説明 |
| --- | --- | --- |
| `--node-config FILE` | `config/node-config.json5` | node、DDS、State、制御の設定 |
| `--zenoh-config FILE` | `config/zenoh-config.json5` | robot nodeのZenoh設定 |

```console
$ uv run node.py --help
```

### Keyboard

| Option | 既定値 | 説明 |
| --- | --- | --- |
| `--keyboard-config FILE` | `examples/keyboard-config.json5` | keyboardの目標値、rate、Zenoh key prefix |
| `--zenoh-config FILE` | `examples/keyboard-zenoh-config.json5` | keyboardのZenoh設定 |

```console
$ uv run examples/keyboard.py --help
```

CLI optionは設定ファイル全体を選択します。
個々の値をcommand lineから上書きすることはできません。

## Robot node設定

管理対象の完全なexampleは[`config/node-config.example.json5`](../../config/node-config.example.json5)です。
未知のfield、必須fieldの欠落、誤った型は拒否されます。

### Top-level設定

| Key | 型 | 説明 |
| --- | --- | --- |
| `zenoh_key_prefix` | concreteなZenoh key | このrobotのcommand keyとState keyのprefix。wildcardは拒否される |
| `node_state_publish_frequency_hz` | 0より大きいHz | 独立したNode State publisherの周波数 |

keyboardの`zenoh_key_prefix`は完全に一致させてください。

### `dds`設定

| Key | 型 | 説明 |
| --- | --- | --- |
| `domain_id` | 0以上の整数 | Unitree channel factoryへ渡すDDS domain |
| `network_interface` | 空でない文字列 | Go2へ接続した専用有線interface名 |
| `sport_mode_state_topic` | 空でない文字列 | DDS `SportModeState` topic。既定contractは`rt/sportmodestate` |
| `first_state_timeout_seconds` | 0より大きい数値 | 最初の有効な`SportModeState`を待つ時間 |
| `state_freshness_seconds` | 0より大きい数値 | 最新の有効なDDS observationをfreshとみなす最大時間 |

### `sport_client`設定

| Key | 型 | 説明 |
| --- | --- | --- |
| `rpc_timeout_seconds` | 0より大きい数値 | 同期SportClient RPCのtimeout |

### `robot_state`設定

| Key | 型 | 説明 |
| --- | --- | --- |
| `quiescent_linear_speed_mps` | 0より大きいm/s | 静止とみなす各計測linear velocity成分の最大絶対値 |
| `quiescent_yaw_rate_rad_s` | 0より大きいrad/s | 静止とみなす計測yaw rateの最大絶対値 |

静止thresholdは、立っているrobotが速度commandを受理できるかどうかの判定には使いません。
Downの確認と、計測運動がthresholdを超えている間に`StandDown()`を送らないために使います。

### `control`設定

| Key | 型 | 説明 |
| --- | --- | --- |
| `velocity_command_timeout_seconds` | 0より大きい数値 | 最新速度requestの最大保持時間。超過後の次の適用可能なStateで速度0の`Move`を1回送り、requestをclearする |
| `shutdown_timeout_seconds` | 0より大きい数値 | 終了時のDown workflowが観測Stateによって完了するまでの上限時間 |

## Keyboard設定

管理対象の完全なexampleは[`examples/keyboard-config.example.json5`](../../examples/keyboard-config.example.json5)です。

### 一般設定

| Key | 型 | 説明 |
| --- | --- | --- |
| `zenoh_key_prefix` | concreteなZenoh key | robot nodeと一致させる |
| `loop_frequency_hz` | 正の整数Hz | keyboard入力、速度publish、画面更新の周波数 |
| `node_state_timeout_seconds` | 0より大きい数値 | Node State publicationがない場合のdashboard表示専用timeout |

`node_state_timeout_seconds`が影響するのはdashboardだけです。
nodeのDDS freshness thresholdではありません。

### `velocity_targets`設定

| Key | 型 | 説明 |
| --- | --- | --- |
| `vx` | 0より大きく2.5以下の数値 | 前後方向keyboard目標値の絶対値 |
| `vy` | 0より大きく1.0以下の数値 | 左右方向keyboard目標値の絶対値 |
| `vyaw` | 0より大きく4.0以下の数値 | yaw rate keyboard目標値の絶対値 |

exampleの目標値は意図的にprotocol上限より低くしています。

### `velocity_ramp_rates`設定

| Key | 型 | 説明 |
| --- | --- | --- |
| `vx_mps2` | 0より大きいm/s² | 1秒あたりの前後command最大変化量 |
| `vy_mps2` | 0より大きいm/s² | 1秒あたりの左右command最大変化量 |
| `vyaw_rad_s2` | 0より大きいrad/s² | 1秒あたりのyaw rate command最大変化量 |

## Keyboard操作

Shiftは移動のdeadmanであり、姿勢requestを送るときにも必要です。

| 入力 | 動作 |
| --- | --- |
| `Shift+W` / `Shift+S` | 正 / 負の`vx` |
| `Shift+A` / `Shift+D` | 正 / 負の`vy` |
| `Shift+Q` / `Shift+E` | 正 / 負の`vyaw` |
| `Shift+R` | Stand requestを1回publish |
| `Shift+F` | Down requestを1回publish |
| `Space` | 速度0をpublish |
| Shiftを離す | 速度0をpublish |
| Windowがfocusを失う | 速度0をpublishし、再arm前にShiftのreleaseを要求 |
| `Esc`またはwindowを閉じる | 速度0をpublishして終了 |

keyboardはローカル速度を設定目標へrampさせます。
実行中は現在速度を各更新でpublishします。
姿勢requestは対応するkey-down eventで1回だけpublishし、retryしません。

## Zenoh keyとcommand contract

`zenoh_key_prefix: "unitree/go2"`の場合、nodeは次のkeyを使います。

| Key | 方向 | 内容 |
| --- | --- | --- |
| `unitree/go2/command` | keyboard/clientからnode | 速度または姿勢のJSON |
| `unitree/go2/state` | nodeからobserver | canonical Node State JSON。Zenoh `get`でも取得可能 |

### 速度command

```json
{"type":"velocity","vx":0.5,"vy":0.0,"vyaw":0.2}
```

| Field | 範囲 |
| --- | --- |
| `vx` | -2.5から3.8 m/s |
| `vy` | -1.0から1.0 m/s |
| `vyaw` | -4.0から4.0 rad/s |

keyboardは速度をdrop congestion control、best-effort reliabilityでpublishし、古いcommandのbacklogより新しい操作を優先します。

### 姿勢command

```json
{"type":"posture","posture":"stand"}
```

```json
{"type":"posture","posture":"down"}
```

姿勢値は`stand`と`down`だけです。
keyboardは姿勢をblock congestion control、reliable deliveryでpublishします。
独立したStop commandはありません。
速度0は`Move(0, 0, 0)`であり、`StopMove()`はnodeのDown workflowだけに属します。

速度と姿勢はそれぞれ最新requestだけを保持し、姿勢を優先します。
処理時点のRobot Stateで許可されないcommandは、将来のStateまで遅延せず破棄します。

## 公開State contract

独立したpublisherはJSON snapshotを`{zenoh_key_prefix}/state`へ定期publishします。
Zenoh `get`は最新のcoherentなNode Stateから直接replyを生成します。

### Top-level field

| Field | 説明 |
| --- | --- |
| `lifecycle` | `starting`、`running`、`shutting_down`、`stopped` |
| `robot_connected` | 最新の有効なDDS observationがfreshか |
| `robot_state` | 最新の分類済みrobot observation、またはUnknown State |
| `accepting_commands` | 速度と姿勢の両方に共通する単一のingress gate |
| `requested_posture` | 最新のbuffer中または実行中の姿勢target。なければ`null` |
| `requested_velocity` | 最新のbuffer中速度。なければ`null` |
| `last_posture_action` | active workflowで最後に送った姿勢SDK step。なければ`null` |
| `last_sdk_diagnostic` | 最新のSDK command診断。なければ`null` |
| `last_node_error` | 最新のnode-level input errorまたはshutdown error。なければ`null` |

`requested_posture`、`last_posture_action`、`last_sdk_diagnostic`はrequestと診断を表します。
これらはrobot姿勢の根拠ではありません。
このcontractにはpublication revisionもprocess-localなmonotonic timestampもありません。

### `robot_state` field

| Field | 説明 |
| --- | --- |
| `state` | `damping`、`down`、`locked_stand`、`ready_stand`、`locomotion`、`unsupported`、`unknown` |
| `reason` | Unknownの場合は`no_sample`、`stale`、`awaiting_state`。それ以外は`null` |
| `motion` | `moving`、`quiescent`、`unknown` |
| `state_machine_code` / `state_machine_name` | Unitree V2.0 motion state machineの識別情報 |
| `mode` / `mode_name` | 粗いSportModeState mode |
| `velocity` | 観測した3成分linear velocity。なければ`null` |
| `yaw_speed` | 観測したyaw speed。なければ`null` |
| `stamp_sec` / `stamp_nanosec` | robotが提供したState timestamp。なければ`null` |

### SDK診断field

`last_sdk_diagnostic`は`command`、任意の`velocity`、任意の整数`code`、任意の`error`を含みます。
0以外のSDK codeは報告されますが、Robot Stateを確定させず、遷移も進めません。
特に`StopMove()`が`-1`を返しても診断に留まり、次に観測したStateが実行可能な処理を決めます。

## Robot Stateごとの動作

| Robot State | 観測条件 | Stand | Down | 速度 |
| --- | --- | --- | --- | --- |
| Damping | state machine 1001、mode 0 | 静止時に`RecoveryStand` | 待機 | 破棄 |
| Down | state machine 1004または2006、mode 5、静止 | `StandUp` | 完了 | 破棄 |
| Locked stand | state machine 1002、mode 0 | `BalanceStand` | Down workflow開始 | 破棄 |
| Ready stand | state machine 100、mode 0/1、または1013、mode 1 | 完了 | Down workflow開始 | `Move` |
| Locomotion | state machine 100/1013、mode 3 | `BalanceStand` | Down workflow開始 | `Move` |
| Unsupported | 上記以外のfreshな組合せ | 新規requestを破棄 | 新規requestを破棄 | 破棄 |
| Unknown | 利用可能な観測Stateなし | 拒否 | 拒否 | 拒否 |

計測motionによって、移動可能なReady standまたはLocomotionを別Stateへ変えることはありません。
motionは、静止したCrouchをDownと確認するためと、`StopMove()`から`StandDown()`まで静止を待つためだけに使います。

action tableとconcurrencyのdecisionは[ADR-0001](../adr/0001-observed-state-driven-control-architecture.md)と[ADR-0002](../adr/0002-shared-node-state-concurrency.md)を参照してください。

## Timingと実行方式

各timeoutとfrequencyの目的は独立しています。

| 設定または定数 | Example値 | 用途 |
| --- | --- | --- |
| DDS publisher frequency | Go2側が決定 | Stateは非同期に到着し、nodeは設定周期でDDSをpollしない |
| Control wait最大値 | 0.05 s | State待機とlifecycle deadline確認中の最大sleep。State到着時は即時wake |
| `dds.state_freshness_seconds` | 0.2 s | control、command ingress、publication、query、shutdownに共通するfreshness境界 |
| `sport_client.rpc_timeout_seconds` | 0.2 s | 同期SportClient RPCの最大待機時間 |
| `control.velocity_command_timeout_seconds` | 0.25 s | 次の適用可能なStateでbuffer中の速度requestを期限切れにし、速度0の`Move`を1回送信 |
| `node_state_publish_frequency_hz` | 20 Hz | 独立した定期Node State publication |
| `node_state_timeout_seconds` | 2.5 s | keyboard dashboard表示専用timeout |

DDS受信は非同期です。
Unitree subscriberの`queueLen=1` reader threadは受信Stateを変換し、local受信時刻とともに最新observationを保存してcontrol workerをwakeします。
workerは未処理の最新observationを分類し、速度より先に姿勢を確認し、そのobservationに対して最大1回のSDK callを送ります。

SportClient callはcontrol workerだけで同期実行されます。
control workerがblockしている間もDDS受信、Zenoh callback、query、定期publicationは継続します。
姿勢effectは先にUnknown Robot Stateのsemanticsをcommitしてcommand ingressを閉じ、次の定期publicationまたはqueryがそのstateを観測します。
call終了後に受信した有効なDDS observationだけがworkflowを継続できます。

## Downとshutdown

対応しているstanding StateからのDown requestは次の順序で処理します。

```text
StopMove once -> newer State -> wait until quiescent
              -> StandDown once -> newer observed Down -> complete
```

SDK return valueによってstepをskipしたり完了扱いにしたりしません。
DampingはDownの確定ではありません。
Down workflowがDampingで待っている間に新しいStand requestを受理すると、Down targetを置き換え、次の適用可能なStateから`RecoveryStand()`を選択できます。

shutdownは通常のcommand ingressを閉じ、同じDown workflowを使います。
観測したDownだけが完了条件です。
先にshutdown deadlineへ達した場合、processがsubscriberを閉じる前に、最後の公開Stateへnode errorを記録します。

## Troubleshooting

### `Timed out waiting for SportModeState`

- Go2の電源が入り、専用有線interfaceがupであることを確認する。
- `dds.network_interface`、`domain_id`、`sport_mode_state_topic`を確認する。
- CycloneDDSが選択したinterfaceを利用できることを確認する。

`dds.first_state_timeout_seconds`までに有効なstartup Stateを受信しない場合、nodeは非0 statusで終了します。

### `robot_connected`がfalse、または`reason`が`stale`

`dds.state_freshness_seconds`以内に有効なDDS Stateが到着していません。
新しいcommandは拒否され、unsafeなbuffer中stateはcontrolによってclearされ、公開Stateにはstaleなobservation値を含みません。
DDS Stateを復旧してください。
最後のSDK resultからrobot姿勢を推定しないでください。

### Robot Stateが`awaiting_state`付きUnknown

姿勢RPC直前・直後の想定された状態です。
RPC完了後に受信した有効なStateまでcommand受付を閉じます。

### Robot StateがUnsupported

State sampleはfreshですが、motion state machineとcoarse modeが対応表にありません。
command ingressは開いたままの場合がありますが、Unsupportedで処理されたcommandは破棄されます。
分類を変更する前に、raw state machine、mode、motion、firmware versionを確認してください。

### Dashboardがwaitingまたはstaleを表示する

robot nodeが動作中であること、両applicationの`zenoh_key_prefix`が一致すること、Zenoh設定が同じnetworkへ参加していることを確認します。
dashboardのstalenessは、`robot_connected`が示すDDS freshnessとは別です。

### `last_sdk_diagnostic.error`がSDK codeまたはexceptionを示す

診断情報として扱います。
次に観測したRobot Stateと実機を確認してください。
return codeだけを根拠に姿勢commandを繰り返さないでください。

## 実機での手動acceptance

低いkeyboard targetを使い、機体を支持し、次を確認するときは公開StateとSDK診断をすべて記録してください。

1. 実機がDownの状態で起動し、State受信を確認する。
2. Standを送り、Ready standが観測されるまで待つ。
3. 設定した各方向へ低速で歩かせ、停止する。
4. Downを送り、`StopMove()`が1回だけ実行され、その後の新しい静止Stateを受けてから`StandDown()`が実行されることを確認する。
5. もう一度Standを送り、robotがDampingを報告した場合はそこからのrecoveryも確認する。
6. nodeを停止し、観測したDownだけでshutdownが完了することを確認する。

実機の挙動と公開Stateが一致しない場合は、無線remoteですぐに停止してください。
