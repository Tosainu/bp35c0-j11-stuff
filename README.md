# bp35c0-j11-stuff

Wi-SUN モジュール [RHOM BP35C0-J11][bp35c0-j11] で遊んだり、ついでに RP2040 とか Rust の組み込み開発事情を調査してるリポジトリ。

- [`bp35c0-j11`](./bp35c0-j11)
    - BS35C0-J11 のコマンド・レスポンスを扱うライブラリ
    - RP2040 でも動かすために `no_std` (要 `alloc`) も想定
    - 現状 B ルート接続とデータ送受信に必要なコマンドにフォーカスしていて、それ以外のコマンドは基本的に未対応
- [`rp2040-log-to-cdc`](./rp2040-log-to-cdc)
    - 後述の RP2040 + BS35C0-J11 の検証環境で、スマートメーターに接続したり取得したデータを USB CDC 経由でダラダラながすやつ
    - モジュールとなにかするだけなら USB-UART 直結で済みそうだけど、いい感じの小物パーツが手元になくてリセットピンとかの制御に悩んだので RP2040 経由で動かすことにした、という経緯
    - 現時点では起動後アクティブスキャンをかけてスマートメーターがいそうなチャンネルを探索、Bルート接続 & PANA 認証、スマートメーターの時刻や瞬時電力を読み出し、という動作をする  
![echonet](https://github.com/user-attachments/assets/b097a747-6711-4857-98f0-15640ad2191b)
- [`rp2040-akizuki-lcd-aqm0802`](./rp2040-akizuki-lcd-aqm0802) (BP35C0-J11 と関係なし)
    - LCD 動作確認と制御方法確認のため実装
- [`rp2040-adt7310`](./rp2040-adt7310) (BP35C0-J11 と関係なし)
    - 温度センサーの動作確認と制御方法確認のため実装
    - 測定値を RP2040 の UART0 に書き出す

## ソースコードのビルドとテスト

リポジトリにあるコードはすくなくとも Rust 1.80.1 (stable) で動きます。ただし [`.cargo/config.toml`](./.cargo/config.toml) でデフォルトのターゲットを `thumbv6m-none-eabi` にしているので、それを追加するか必要に応じて `--target <TRIPLE>` を渡します。また [rp-rs/rp2040-project-template][rp2040-template] にならって [flip-link][flip-link] を指定しているので、こちらも必要です。[rustup][rustup] がインストールされている場合、例えば次の手順でビルドできます。

    $ rustup target add thumbv6m-none-eabi
    $ cargo install flip-link
    $ cargo build

テストコードはホストの環境で直接実行できます。例えば次のコマンドを実行すると、`.cargo/config.toml` の設定関係なしにホスト向けにテストコードがビルド・実行されるはずです。

    $ cargo test --target "$(rustc -vV | grep host | cut -d' ' -f2)"

### Bルート認証ID 等の埋め込み

`rp2040-log-to-cdc` は、Bルート認証ID とパスワードをバイナリに埋め込むため [`build.rs`](./rp2040-log-to-cdc/build.rs) でゴニョゴニョしています。埋め込む値を変更するには、`rp2040-log-to-cdc/secret.txt` を作成し、1行目に Bルート認証ID、2行目にパスワードを記述してビルドしなおします。

### ターゲット RP2040 基板の変更

`rp2040-log-to-cdc` は、複数の RP2040 基板への対応を Cargo の [features][cargo-features] でできないか実験しています。現時点では [Adafruit Feather RP2040][feather-rp2040] (デフォルト) と Raspberry Pi Pico に対応しています。 例えば次のオプションを渡してビルドすると Raspberry Pi Pico 向けになります。なお、Raspberry Pi Pico は私が未所持のため動作未確認です。

    $ cargo build --features rp2040-log-to-cdc/rp-pico --no-default-features

モジュールとの接続方法等は後述します。

## RP2040 で動かす

動作確認では RP2040 基板に [Pink Adafruit Feather RP2040][pink-feather-rp2040] を使っています。また、BP35C0-J11 のために [SPRESENSE-WiSUN-EVK-701][spresense-wi-sun-add-on] を使っています。SPRESENSE-WiSUN-EVK-701 は名前の通り SPRESENSE 向け拡張ボードですが、リンク先にある回路図を見ると実質レベルシフターとチップアンテナ付きの BP35C0-J11 評価ボードなのがわかります[^evk-701-note]。ちなみに下調べが甘かったなという反省点として、この基板には 2.54mm ピッチのピンヘッダがついているものの足が短いのでブレッドボードに向かなかったというのがあります。浮いてきてしまうので写真の通りテープ固定しています。

[^evk-701-note]: もちろんメーカーの想定した使い方ではなく、この使い方を推奨するものではありません。自己責任でおおねがいします。

![](https://github.com/user-attachments/assets/373401e0-0ac9-459d-88a2-cf884d201006)

### 接続

Feather RP2040 と SPRESENSE-WiSUN-EVK-70 は次のように接続しています。ユニバーサル基板実装にともない、ピン割り当てを上記画像のものから変更しています。

| Feather RP2040 | SPRESENSE-WiSUN-EVK-70 | メモ |
| --- | --- | --- |
| `D24` | `UARTT2_TX` (J-2 - 2) | |
| `D25` | `UARTT2_RX` (J-2 - 3) | |
| `D11` | `GPIO1` (J-4 - 4) | BP35C0-J11 の `RESETN` にレベルシフター TXS0108E を介して接続されている |
| `D10` | `GPIO2` (J-4 - 5) | TXS0108E の `OE` に接続されている |
| `3.3V` | `3.3V` (J-1 - 1) | TXS0108E の `VCCB` (BP35C0-J11 側) の電源 |
| `3.3V` | `1.8V` (J-4 - 2) | TXS0108E の `VCCA` (SPRESENSE 側) の電源 (今回は RP2040 とつなぐので 3.3 V で問題なし) |
| `GND` | `GND` (J-2 - 1) | SPRESENSE-WiSUN-EVK-70 側は `GND` (J3 - 1) でも可 |

Raspberry Pi Pico 向けにビルドした場合の接続は次を想定しています。

| Raspberry Pi Pico | SPRESENSE-WiSUN-EVK-70 |
| --- | --- |
| `GP4` | `UARTT2_TX` (J-2 - 2) |
| `GP5` | `UARTT2_RX` (J-2 - 3) |
| `GP10` | `GPIO1` (J-4 - 4) |
| `GP11` | `GPIO2` (J-4 - 5) |
| `3V3V(OUT)` | `3.3V` (J-1 - 1) |
| `3V3V(OUT)` | `1.8V` (J-4 - 2) |
| `GND` | `GND` (J-2 - 1) |

### Bus Blaster v3 で SWD

変更したプログラムを RP2040 で動かすために毎回 USB ケーブルつなぎなおしてるのが面倒になったのと、流行りの [probe-rs][probe-rs] にあこがれて、[このとき][bus-blaster-blog]買っていた JTAG デバッガー [Bus Blaster v3](http://dangerousprototypes.com/docs/Bus_Blaster_v3_design_overview) を出してきました。

![](https://github.com/user-attachments/assets/97ed3ea5-6ab5-46a6-8e4d-32c995282985)

Bus Blaster v3 を KT-link 互換にするたんめ、[このリポジトリ][bus-blaster-ktlink]の `system.svf` を CPLD に書き込みました。Feather RP2040 の裏面パッドからひっぱってきた SWD のポートと接続するとあっさり OpenOCD で認識。GDB でロードやデバッグ、などもできました。

    $ openocd --version
    Open On-Chip Debugger 0.12.0
    Licensed under GNU GPL v2
    For bug reports, read
    	http://openocd.org/doc/doxygen/bugs.html

    $ openocd -f interface/ftdi/dp_busblaster_kt-link.cfg -c 'adapter speed 2500' -f target/rp2040.cfg
    Open On-Chip Debugger 0.12.0
    Licensed under GNU GPL v2
    For bug reports, read
    	http://openocd.org/doc/doxygen/bugs.html
    adapter speed: 2500 kHz
    
    Info : FTDI SWD mode enabled
    Info : Listening on port 6666 for tcl connections
    Info : Listening on port 4444 for telnet connections
    Info : clock speed 2500 kHz
    Info : SWD DPIDR 0x0bc12477, DLPIDR 0x00000001
    Info : SWD DPIDR 0x0bc12477, DLPIDR 0x10000001
    Info : [rp2040.core0] Cortex-M0+ r0p1 processor detected
    Info : [rp2040.core0] target has 4 breakpoints, 2 watchpoints
    Info : [rp2040.core1] Cortex-M0+ r0p1 processor detected
    Info : [rp2040.core1] target has 4 breakpoints, 2 watchpoints
    Info : starting gdb server for rp2040.core0 on 3333
    Info : Listening on port 3333 for gdb connections
    Info : starting gdb server for rp2040.core1 on 3334
    Info : Listening on port 3334 for gdb connections

と、ここで probe-rs は FTDI ベースのデバッガーの SWD に (少なくとも現時点では) 対応していないのに気づきます。残念。また GDB も、semihosting/RTT などを試していくとプログラムによっては挙動があやしい (?) ことがあり、結局このリポジトリの開発で採用するのは断念。

それでも SWD でプログラムをロードさせるのは諦めきれず、いろいろ調査のうえ OpenOCD 単体で書き込めるのがわかりました。また、`.cargo/config.toml` から [`runner`][cargo-runner] を次のように設定して、`cargo run` で OpenOCD が立ち上がって書き込み & 実行できました。

```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = [
  "sh",
  "-c",
  """openocd \
    -f interface/ftdi/dp_busblaster_kt-link.cfg \
    -c \"adapter speed 2500\" \
    -f target/rp2040.cfg \
    -c \"program $0 verify reset exit\"""",
]
```

## ユニバーサル基板実装

ユニバーサル基板実装に実装しました。ついでに眠らせていた LCD と温度センサーを空いたスペースに実装しました。

![DSC_3020-Edit](https://github.com/user-attachments/assets/86eeea2a-759b-4d7c-aec3-c6f8b27b8ed1)

### パーツリスト

Feather RP2040 と SPRESENSE-WiSUN-EVK-701 はリストから除いています。

| パーツ | 仕様 | 個数 | メモ |
| --- | --- | --: | --- |
| ユニバーサル基板 | 片面 95×72mm | 1 | 秋月 [103230](https://akizukidenshi.com/catalog/g/g103230/) |
| ピンソケット | 1x12 pin | 1 | 秋月 [110101](https://akizukidenshi.com/catalog/g/g110101/), Feather RP2040 用 (1) |
| ピンソケット | 1x16 pin | 1 | 秋月 [110103](https://akizukidenshi.com/catalog/g/g110103/), Feather RP2040 用 (2) |
| 低背ピンソケット | 1x14 pin | 2 | 秋月 [100661](https://akizukidenshi.com/catalog/g/g100661/), SPRESENSE-WiSUN-EVK-701 用にカットせず使用 少しキツかった, 理想は[これ](https://www.connect.co.jp/products/detail/1434) |
| ピンヘッダー | 1xN | 適量 | デバッグ用との電源や UART 引き出し用 |
| LCD | I2C 接続 8x2行 | 1 | 秋月 [109109](https://akizukidenshi.com/catalog/g/g109109/) 相当, ドライバー IC として ST7032 を搭載 |
| 温度センサー | SPI 接続 | 1 | 秋月 [106708](https://akizukidenshi.com/catalog/g/g106708/) 相当, ADT7310 を搭載 |

### 接続

追加の LCD と温度センサー以外は前述したとおり。自分の作業メモ程度のものであるうえ、その時の都合からペイント系ソフトで描いています。ちゃんとやるならベクター形式で扱いたいところ。背景は秋月の寸法図から引用したものです[^quote-akizuki]。

![rp2040-bp35c0-j11](https://github.com/user-attachments/assets/6aea4c24-fff2-4376-8abf-084f22794535)

[^quote-akizuki]: 非商業的かつ個人的な目的での「使用」のため、この引用は大丈夫なはず… ただし色の変更やパターンの書き込みが「改変」にあたらないかは不安 https://akizukidenshi.com/catalog/pages/terms_of_service.aspx

## 今後の展望

- [x] ブレッドボードにテープ固定つらい → 基板おこしちゃう？
    - 基板おこすのずっとやってみたいなーとおもいつつ、初めてにちょうどいい難易度の題材がなくできていなかった これならよさそう
    - ただ簡単すぎて、2.54mm ピッチのパーツしかないしユニバーサル基板でいいのでは…? ってなりつつある
- [ ] ECHONET Lite 関連のコード追加してエンコード・デコードする
- [ ] 最終的にスマートメーターから読んだデータをおうち Grafana に流し込みたい

## 参考資料

そのほか開発にあたって参考にしたページや資料のリスト:

- [BP35C0-J11 - データシートと製品詳細 | ローム株式会社 - ROHM Semiconductor][bp35c0-j11]
    - BP35C0-J11 UART IF 仕様書
    - BP35C0-J11 Bルート通信について
- [SPRESENSE Add-on Boards - Wi-SUN & Sensors & Bluetooth® LE | ローム株式会社 - ROHM Semiconductor][spresense-wi-sun-add-on]
    - ハードウェア関連資料 - 回路図
- [規格書・仕様書など | ECHONET][echonet]
    - ECHONET Lite規格書 Ver.1.14 - 第2部 ECHONET Lite 通信ミドルウェア仕様
        - ECHONET Lite のフレーム構造の解説がある
    - アプリケーション通信インタフェース仕様書 - 低圧スマート電力量メータ・コントローラ間 Ver.1.10
        - スマートメーター関連の情報がある
    - APPENDIX ECHONET機器オブジェクト詳細規定Release R rev.2
        - スマートメーター含む、ECHONET 機器から読み出せる値 (オブジェクト) の詳細について書かれている
- [rp2040_hal - Rust][rp2040-hal]
- [bgamari.github.com - SWD with OpenOCD and a Bus Blaster][bus-blaster-swd]
- [SWD support for FTDI adapter based debugging · Issue #2557 · probe-rs/probe-rs][probe-rs-2557]

## ライセンス

本リポジトリの内容は [MIT License](./LICENSE) のもとで公開しています。

本リポジトリは [rp-rs/rp2040-project-template][rp2040-template] をベースに作成しました。

> [MIT][rp2040-template-license]
> ```
> MIT License
> 
> Copyright (c) 2021 rp-rs organization
> 
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
> 
> The above copyright notice and this permission notice shall be included in all
> copies or substantial portions of the Software.
> 
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.
> ```

[bp35c0-j11]: https://www.rohm.co.jp/products/wireless-communication/specified-low-power-radio-modules/bp35c0-j11-product
[spresense-wi-sun-add-on]: https://www.rohm.co.jp/support/spresense-add-on-board#anc-04
[feather-rp2040]: https://www.adafruit.com/product/4884
[pink-feather-rp2040]: https://www.adafruit.com/product/5299
[rustup]: https://rustup.rs/
[cargo-features]: https://doc.rust-lang.org/cargo/reference/features.html
[cargo-runner]: https://doc.rust-lang.org/cargo/reference/config.html#targettriplerunner
[bus-blaster]: http://dangerousprototypes.com/docs/Bus_Blaster_v3_design_overview
[bus-blaster-blog]: https://myon.info/blog/2020/07/05/bus-blaster/
[bus-blaster-ktlink]: https://github.com/bharrisau/busblaster
[bus-blaster-swd]: https://bgamari.github.io/posts/2014-08-23-swd-with-busblaster-and-openocd.html
[probe-rs-2557]: https://github.com/probe-rs/probe-rs/issues/2557
[probe-rs]: https://github.com/probe-rs/probe-rs
[echonet]: https://echonet.jp/spec_g/
[rp2040-template]: https://github.com/rp-rs/rp2040-project-template/tree/4cfa8a7d7b2991f77d85199b9d65296d54ced071
[rp2040-template-license]: https://github.com/rp-rs/rp2040-project-template/blob/4cfa8a7d7b2991f77d85199b9d65296d54ced071/MIT
[rp2040-hal]: https://docs.rs/rp2040-hal/latest/rp2040_hal/
[flip-link]: https://github.com/knurling-rs/flip-link
