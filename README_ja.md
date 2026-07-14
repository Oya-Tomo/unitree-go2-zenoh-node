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
       - 250 ms watchdog
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
具体Zenoh keyへ変更できます。

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

運用上は実機を伏せ姿勢から始めます。ただしnodeはtelemetryなしに姿勢を断定
しません。起動時に`StopMove()`を呼び、姿勢`unknown`を公開し、明示的なstandが
成功するまで速度を無視します。

```text
unknown/down -- stand --> standing_up -- 3 s + BalanceStand --> standing
standing    -- down  --> standing_down -- StandDown ----------> down
```

- Stand: `StopMove()` → `StandUp()` → 設定時間（既定3秒）→
  `BalanceStand()` → 歩行を有効化します。
- Down: 歩行を無効化 → `StopMove()` → `StandDown()`です。
- 姿勢が`standing`かつ`walking_enabled=true`のときだけ速度をSDKへ渡します。
- monotonic clockの250 ms watchdogは、最後に渡した速度が古くなると
  `StopMove()`を1回だけ呼びます。新しい有効速度で再開し、無効messageでは
  watchdogを更新しません。
- `dds.rpc_timeout_seconds`はwatchdog時間以下でなければならず、例では同期SDK
  callの停滞を抑えるため200 msです。
- 正常な`SIGINT`/`SIGTERM`終了時は`StopMove()` → 1秒待機 → `StandDown()`の後で
  Zenoh resourceを閉じます。
- process強制終了、host停止、電源断ではdown動作を保証できません。

## 状態Key

状態変化をZenoh `put`で配信し、各keyにexact Queryableを宣言します。後から接続
したclientは`get`で現在値を取得できます。

| Key | 意味 |
| --- | --- |
| `{robot_key}/state/command/requested` | 受信した最後の有効速度。歩行禁止中のcommandも記録 |
| `{robot_key}/state/command/applied` | SDKへ正常に転送できた最後の速度 |
| `{robot_key}/state/posture` | `unknown`, `standing_up`, `standing`, `standing_down`, `down` |
| `{robot_key}/state/health` | node状態と安全flag |

```json
{"vx":0.5,"vy":0.0,"vyaw":0.2,"active":true}
```

```json
{
  "status": "ready",
  "walking_enabled": true,
  "watchdog_triggered": false,
  "last_error": null
}
```

`applied`はSDK callが成功したことを意味し、実機telemetryで確認した物理速度・姿勢
ではありません。

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
| windowがfocusを失う | ゼロ速度をpublishし、Shiftを一度離すまで再armしない |
| `Esc` / windowを閉じる | ゼロ速度をpublishして終了 |

複数軸を同時操作できます。keyboard nodeは設定targetまでrampしながら20 Hzで
publishし、requested/applied速度、姿勢、walking enable、watchdog、health、
deadmanを表示します。
速度はbest-effort/drop QoSです。stand/downは同じcommand key上の別の
reliable/blocking publisherを使い、報告姿勢がtargetへ到達するか、設定timeoutを
operator errorとして表示するまで再送します。

## 開発時の検証

次は実機なしで実行できます。

```bash
uv run ruff format --check .
uv run ruff check .
uv run pyright
uv run python -m unittest discover -s tests -v
```

合格しても、管理された環境での実機smoke testは別途必要です。
