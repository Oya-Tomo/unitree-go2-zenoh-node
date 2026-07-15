# Unitree Go2 Zenoh Node

[English](README.md)

Zenohで受け取った高レベルコマンドをUnitree Go2の`SportClient`へ渡す、
安全側の制御ノードです。1プロセスが1本のDDSネットワークインターフェースを
通して1台のロボットを制御します。

> [!WARNING]
> このソフトウェアは実機を動かします。最初は機体を支持または浮かせ、無線
> リモコンと緊急停止手順を用意し、使用中の機体・firmwareで挙動を確認して
> ください。

## 構成

```text
高レベルポリシー / pygameキーボードnode
                  |
                  | Zenoh JSON command (20 Hz)
                  v
       unitree-go2-zenoh-node
       - Pydantic validation
       - 姿勢状態機械
       - best-effort 250 ms watchdog
       - 状態のput / Queryable
                  |
                  | Unitree SDK2 / CycloneDDS
                  v
               Go2 1台
```

配置単位は **1 nodeプロセス + 1 NIC + 1 robot** です。別のロボットには別の
プロセス、設定、NIC、固有の`robot_key`を使います。MVPでは1つのSDKプロセス
から複数台を選択せず、複数publisher間の所有権調停も行いません。

## 必要環境とインストール

- Python 3.13
- [uv](https://docs.astral.sh/uv/)
- ローカルでビルドしたCycloneDDS（本リポジトリでは
  `~/Packages/cyclonedds/install`を想定）
- 設定したNICから到達できるUnitree Go2

```bash
git clone --recurse-submodules https://github.com/Oya-Tomo/unitree-go2-zenoh-node.git
cd unitree-go2-zenoh-node
git submodule update --init --recursive

export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
uv sync --all-groups
```

`unitree-sdk2py`は`third_party/unitree_sdk2_python`のeditableなuv path source
として登録済みです。submoduleは`Oya-Tomo/unitree_sdk2_python`の
`chore/cdds-py313`ブランチを追跡します。

## 設定

実行時の値はJSON5へ置き、CLIでは設定ファイルのパスだけを選びます。

```bash
cp config/node-config.example.json5 config/node-config.json5
cp config/zenoh-config.example.json5 config/zenoh-config.json5
cp examples/keyboard-config.example.json5 examples/keyboard-config.json5
cp examples/keyboard-zenoh-config.example.json5 examples/keyboard-zenoh-config.json5
```

`dds.network_interface`をロボット接続用NICへ変更してください。robot nodeと
command publisherの`robot_key`は一致させます。例は`unitree/go2`ですが、任意の
具体Zenoh keyへ変更できます。nodeは`state_heartbeat_seconds`ごとに全stateを
再publishし、keyboardは`state_stale_after_seconds`の間に新しいheartbeatが
届かなければstateをstale表示します。

`dds.sport_mode_state_topic`はraw DDSの`SportModeState` topicです。既定値は
`rt/sportmodestate`ですが、firmware構成によっては`lf`付きtopicを使うため、実機が
publishするtopicへ合わせてください。
`dds.sport_mode_state_startup_timeout_seconds`は起動時の安全判定前に最初のsampleを
待つ期限です。期限内に届かなければ、motion未受信として安全側の`StopMove()`を
呼びます。`safety.motion_state_max_age_seconds`はsampleを
信頼できる最大経過時間、`safety.balance_confirmation_timeout_seconds`は
`BalanceStand()`後に新しいsampleを待つ期限です。古いsample、未受信、error付き、
矛盾する状態では`StopMove()`を省略せず、歩行も有効化しません。

Zenohの例はpeer mode、multicast discovery、固定listen endpoint
`tcp/0.0.0.0:7447`です。router、TLS、認証、認可は設定していません。信頼できる
ローカルネットワークだけで使用してください。例はロボット1台を想定します。

## 実行

先にロボット側nodeを起動します。

```bash
export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
uv run node.py \
  --node-config config/node-config.json5 \
  --zenoh-config config/zenoh-config.json5
```

次に操作PCでpygame nodeを起動します。

```bash
uv run --group example python -m examples.keyboard \
  --keyboard-config examples/keyboard-config.json5 \
  --zenoh-config examples/keyboard-zenoh-config.json5
```

1台のロボットに対して、同時に有効な速度command publisherは1つだけにして
ください。同じ`robot_key`へpygameと高レベルポリシーを同時接続しないで
ください。

## Command仕様

`{robot_key}/command`へ`application/json`でpublishします。

```json
{"type":"velocity","vx":0.5,"vy":0.0,"vyaw":0.2}
```

```json
{"type":"posture","posture":"stand"}
```

```json
{"type":"posture","posture":"down"}
```

`down`は犬のおすわりに相当する`Sit()`ではなく、伏せ姿勢の`StandDown()`を
呼びます。

### 公式の速度範囲

[Unitree Go2 sport service公式仕様](https://support.unitree.com/home/en/developer/sports_services)
に従い、`Move(vx, vy, vyaw)`を次の範囲で検証します。

| Field | Range | Unit |
| --- | ---: | --- |
| `vx` | `[-2.5, 3.8]` | m/s |
| `vy` | `[-1.0, 1.0]` | m/s |
| `vyaw` | `[-4.0, 4.0]` | rad/s |

これは検証上限であり、推奨速度ではありません。keyboardの既定targetは安全側の
`0.5 m/s`、`0.3 m/s`、`1.0 rad/s`です。

公式仕様では`Move`は無補間で、最後のcommandを1秒間維持します。本nodeは有効な
policy出力を平滑化もclampもせずSDKへ渡します。各publisherが自分の出力をfilter
し、速度を20 Hzでpublishしてください。

## 安全状態機械

運用上は実機を伏せ姿勢から始めます。nodeはGo2の`SportModeState`を購読し、
[Unitree公式のmode定義](https://github.com/unitreerobotics/unitree_ros2)にある
`mode=5`（`lieDown`）をfreshかつ`error_code=0`のsampleで確認した場合だけ、実機が
すでに伏せていると判断します。それ以外では起動時に`StopMove()`を呼び、姿勢
`unknown`を公開し、明示的なstandが成功するまで速度を無視します。起動時のstop
失敗はnodeを歩行禁止に保ったまま再試行します。

```text
unknown/down -- stand --> standing_up -- 3 s + BalanceStand + mode 1観測 --> standing
standing    -- down  --> standing_down -- StandDown --------------------> down
```

- Stand: freshなtelemetryで`lieDown`を確認済みなら`StopMove()`を省略して
  `StandUp()`を呼びます。それ以外では先に`StopMove()`を要求します。その後は
  設定時間（既定3秒）→ `BalanceStand()` → それより新しいfreshな`balanceStand`
  sample確認 → 歩行有効化の順です。設定時間内に確認できなければ歩行を無効のまま
  stopを要求します。公式appですでにfreshな`balanceStand`になっている場合は、
  明示的なstand要求で重複する姿勢callを発行せず、その状態を採用できます。
- Down: 歩行を無効化 → `StopMove()` → `StandDown()`です。stop失敗中はdown遷移を
  保留して再試行し、stop成功を確認する前に`StandDown()`を呼びません。freshな
  `lieDown`を確認した場合は、重複する両SDK callを発行せずstop完了とします。
- appやremoteから外部操作されて`lieDown`を観測した場合も、applied速度をzeroへ
  戻し、姿勢`down`を公開し、歩行を無効化します。ただしnode自身が開始した
  `StandUp()`中は、その直前のdown sampleで同じ遷移を打ち消しません。RPC timeoutや
  例外は実行結果不明として扱い、期待した遷移の観測、明示的な回復barrier、または
  遷移時間より長く続く複数のfreshな観測が得られるまで直前modeを再採用しません。
- node内の推定姿勢だけを根拠にstopを省略しません。以前`down`と推定していても、
  telemetryが矛盾、未受信、staleなら、新しいdown要求は安全側の`StopMove()`経路を
  通ります。
- 保留姿勢は再生queueではなく目標状態として扱います。control loopの1回のdrainで
  1目標だけを処理し、保留中の`down`は後続の`stand` burstで上書きされません。
  `down`のdrain後は、新しい明示的な`stand`を受け付けます。
- 姿勢が`standing`、`walking_enabled=true`で、freshかつerrorなしのtelemetryが
  `balanceStand`または`locomotion`のときだけ速度をSDKへ渡します。未受信、stale、
  error付き、歩行不能modeでは歩行を無効化して`StopMove()`を要求し、stop失敗時は
  degradedの保留状態を維持します。
- 最後にSDKへ渡した速度が古くなるとmonotonic watchdogが`StopMove()`を呼びます。
  stop失敗中は速度を禁止し、確認できるまで一定間隔で再試行します。
  `watchdog_triggered`はこのtimeoutだけを表し、通常のSDK failureには使いません。
- watchdogはcontrol loop上のbest-effortな閾値で、hard real-time deadlineでは
  ありません。SDK callは同期的で同じthreadをblockします。
  `dds.rpc_timeout_seconds`はSDK requestの各phaseを制限しますが、replyを伴う
  1 callに複数phaseがあり、watchdog処理が遅れる場合があります。
- 正常な`SIGINT`/`SIGTERM`終了時、freshな`lieDown`を確認済みなら重複するmotion
  callを発行しません。それ以外では`StopMove()`を試し、1秒待ち、失敗時だけ
  もう1回試します。stop成功を確認した場合だけ`StandDown()`を呼びます。
- process強制終了、host停止、電源断ではdown動作を保証できません。

### 管理された実機smoke test

機体を支持し、remote stopを使える状態で、dashboardのmotion modeとlast errorを
記録しながら次を確認します: 伏せ状態でnode起動 → `Shift+R` → 小さい歩行command
→ `Shift+F` → 再度`Shift+R` → `Shift+F` → node終了。伏せでは`mode=5`、standでは
freshな`balance_stand`観測後にだけ`walking_enabled=true`となるのが期待値です。
motionがstaleのままなら、動かす前に`dds.sport_mode_state_topic`を確認してください。

## 状態Key

状態変化をZenoh `put`で配信し、定期heartbeatとしても再配信し、各keyにexact
Queryableを宣言します。後から接続したclientは`get`で現在値を取得でき、subscriber
はnodeのstateがstaleかローカルに判定できます。

| Key | 意味 |
| --- | --- |
| `{robot_key}/state/command/requested` | 受信した最後の有効速度。歩行禁止中のcommandも記録 |
| `{robot_key}/state/command/applied` | 現在の指令速度推定値。成功した最後の`Move`で更新し、`StopMove`成功後または`lieDown`観測時にzeroへ戻す |
| `{robot_key}/state/posture` | controller姿勢: `unknown`, `standing_up`, `standing`, `standing_down`, `down` |
| `{robot_key}/state/motion` | Go2で最後に観測したmode、mode名、error code、ローカルfreshness判定 |
| `{robot_key}/state/health` | node状態と安全flag |

```json
{"vx":0.5,"vy":0.0,"vyaw":0.2}
```

```json
{"mode":5,"mode_name":"lie_down","error_code":0,"fresh":true}
```

```json
{
  "status": "ready",
  "walking_enabled": true,
  "watchdog_triggered": false,
  "last_error": null
}
```

`applied`はSDK return codeの成功から推定し、telemetryで`lieDown`を確認した場合も
zeroへ戻す指令値です。物理速度の実測値ではありません。`posture`はcontroller状態、
`motion`はそれとの整合に使う実機観測値として分離しています。

## Pygame操作

Shiftは移動のdeadmanで、姿勢commandにも必要です。

| Input | Action |
| --- | --- |
| `Shift+W` / `Shift+S` | `vx`正 / 負 |
| `Shift+A` / `Shift+D` | `vy`正 / 負 |
| `Shift+Q` / `Shift+E` | `vyaw`正 / 負 |
| `Shift+R` | stand後に歩行可能なbalance modeへ移行 |
| `Shift+F` | `StandDown()`で伏せる |
| `Space` | 即座にゼロ速度をpublish |
| `Shift`を離す | 即座にゼロ速度をpublish |
| windowがfocusを失う | ゼロ速度をpublishし、stand再送を取消し、Shiftを一度離すまで移動・姿勢commandを再armしない |
| `Esc` / windowを閉じる | ゼロ速度をpublishして終了 |

複数軸を同時操作できます。keyboard nodeは設定targetまでrampしながら20 Hzで
publishし、requested/applied速度、controller姿勢、実機motion観測、walking
enable、watchdog、health、deadmanを表示します。
速度はbest-effort/drop QoSです。stand/downは同じcommand key上の別の
reliable/blocking publisherを使い、報告姿勢がtargetへ到達するか、設定timeoutを
operator errorとして表示するまで再送します。ACKにはrequestより新しいstate sample
だけを使います。focus喪失、Shift解放、終了では保留`stand`再送を取消します。
すでに要求した安全側の`down`はkeyboard nodeが動作中なら再送対象に残します。

## 開発時の検証

次は実機なしで実行できます。

```bash
uv run ruff format --check .
uv run ruff check .
uv run pyright
uv run python -m unittest discover -s tests -v
```

合格しても、管理された環境での実機smoke testは別途必要です。
