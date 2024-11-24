# StampFlyの操作方法

## 安全に飛ばすために
### 電池の管理
- 専用充電器を用いて充電してください。
- 膨らんだ場合は、塩水等につけて１週間ほどおき完全に放電してから、お住まいの自治体に処理方法を問い合わせてください。
- StampFlyに電池を差し間違えると回路が焼損し、電池も発火の危険がありますので慎重に差し込んでください。
### 安全メガネの着用
- プロペラを覗き込むような場合や、プロペラが緩くすぐ外れるような場合はプロペラが飛んで目に入り失明する恐れがありますから、安全メガネを着用してください。

## 使い方
### 電池の挿入
- ATOMJoyとStampFlyに電池を挿入してください。
- オスとメスの端子に合うように差し込んでください。
- StampFlyに電池を取り付ける際に電池アダプタと機体の端子をずらして刺してしまうと、電池のアダプタから煙が出て焦げ付くか、電池が発火する可能がありますので注意してください。
### Peering (Pairing)
初めて飛ばす時は、ピアリング（ペアリング）をしてください。
- ATOMJoyの液晶パネルを押しながらスイッチを入れる。
- StampFlyに電池を接続する。
- AtomJoyの液晶パネルの表示が変わったらピアリング完了です。
### 飛行可能になるまで
- 電池を接続するとLEDが白く点灯します。
- LEDが白く点灯しているうちに水平な面においてください。
- しばらくするとLEDが紫色になります。紫色の間にセンサのオフセットを取得しているの触らないでください。
- 最後にLEDがイルミネーション表示に変わり飛行可能となります。
### 手動での飛ばし方(スタビライズモード)
- デフォルトではモード２です。以下はモード２での説明です。モード３では左右反対です。
- デフォルトでスタビライズモードです。姿勢を自動的に保ちます。
- 左のスティックを押し込むとStampFlyのLEDが黄色になりArming状態になります。
- 左のスティックを上に倒すとプロペラの回転数が上がり、上昇します。
- 左のスティックを下げるとプロペラの回転数が下がり、下降します。
- 左のスティックの中立でプロペラが止まります。
- 左のスティックの中立より下は意味はありません。
- 左のスティックを左右に倒すと倒した方に機首が方向を変えます。
- 右のスティックの前後左右で機体がその方向に移動します。
- 右のスティックの倒した量に応じて機体が傾きます。
- 再度、左のスティックを押し込むとDisArmingとなりモータが停止しLEDがイルミネーション状態に戻ります。
- 飛行中に、左のスティックを押し込むとモータが止まり落下しますので注意してください。
### 電池切れサイン
- LEDの色が水色になったら電池切れサインです。もうしばらくは飛べますが、続けるとリセットがかかり落下します。電池を充電しましょう。
### 宙返り
- 空中に静止したホバリング状態を維持して、右のスティックを押し込むと自動的に宙返りをします。
### アクロモード
- ATOMJoyの前方にある右ボタンを一度押すとアクロモードに切り替わります。もう一度押すとスタビライズモードに戻ります。ATOMJoyの液晶画面でモードは確認できます。
- アクロモードは姿勢を自動的に維持しません。自分で姿勢を保つように操縦します。
- 右スティックの倒し量に応じた回転速度で機体が回転します。中立に戻すと回転速度０です。回転を止めても傾きはそのままなので自分で戻す必要があります。
### 高度維持モード（開発中）
- StampFlyは全て常に開発中ですが、高度制御は未完成です。
- 着陸した状態で、ATOMJoyの前方の左のボタンを１度押すと高度維持モードになります。
- 高度維持モードは、離陸当初はマニュアル操縦と同じです。高度をゆっくり上げると途中でテイルのLEDがピンク色に変わり高度維持モードを示します。
- 高度維持モードになったら、一度、左スティックを中立に戻してください。
- 中立に戻すと、左ステイックの上下で高さを徐々に変えることができます。
- 高度維持モードは高さの変化は緩やかで素早く高さを変化させることはできません。
- 床面が段差となり、機体と床面との高さが変化すると、それに応じて高度を調整しようとします。
- あまり急激な高度変化が伴うと、機体が勢いよく運動し不安定になるか、リセットがかかって墜落します。
- 着陸は徐々に硬度を下げ、低くなったら左スティックを押し込みDisArmingにしてください。
### モード３
- 開発者のお気に入りモードです。
- ATMJoyの前方の左のボタンを押しながらATOMJoyのスイッチを入れるとモード３になります。スイッチを切るまで変えられません。
- モード３はモード２のスティック操作が左右逆です。ただし、前方ボタンの役割は入れ替わりません。
### 衝撃停止機能
- 大きな衝撃が加わると自動的にモータが停止します。

# StampFly Operation Manual

## Flying Safely

### Battery Management
- Use the dedicated charger for charging.
- If the battery swells, submerge it in saltwater for about a week to fully discharge it, then consult your local municipality for disposal methods.
- Be cautious when inserting the battery into the StampFly. Incorrect insertion can damage the circuit and pose a fire hazard.

### Wearing Safety Glasses
- When inspecting the propeller or if the propeller is loose, wear safety glasses to prevent potential eye injury from a detached propeller.

## How to Use

### Inserting the Battery
- Insert the battery into both ATOMJoy and StampFly.
- Ensure the terminals are aligned correctly.
- Misaligning the battery adapter and the terminal on the StampFly can cause smoke, charring, or a potential fire hazard.

### Peering (Pairing)
Pairing is required when flying for the first time:
- Press and hold the LCD panel button on ATOMJoy while turning it on.
- Connect the battery to StampFly.
- When the display on the ATOMJoy LCD panel changes, pairing is complete.

### Getting Ready for Flight
- When the battery is connected, the LED lights up white.
- Place it on a flat surface while the LED is white.
- After a while, the LED turns purple. During this time, the sensor offsets are being obtained, so do not touch it.
- Finally, the LED changes to an illumination display, indicating that it is ready for flight.

### Manual Flying (Stabilize Mode)
- The default is Mode 2. The following instructions are for Mode 2. In Mode 3, left and right controls are reversed.
- The default mode is Stabilize Mode, which automatically maintains posture.
- Push down the left stick to turn the StampFly LED yellow, indicating Arming mode.
- Pushing the left stick up increases propeller speed, causing ascent.
- Pushing the left stick down decreases propeller speed, causing descent.
- The propellers stop when the left stick is in the neutral position.
- Moving the left stick left or right rotates the drone in that direction.
- Moving the right stick forward, backward, left, or right moves the drone in that direction.
- The degree of tilt of the drone corresponds to the amount of deflection of the right stick.
- Pressing the left stick again disarms the motors and the LED returns to the illumination state.
- Be careful, as pressing the left stick during flight will stop the motors and cause the drone to fall.

### Battery Low Sign
- If the LED turns light blue, it indicates a low battery. The drone can still fly for a while, but it will reset and fall if you continue. Charge the battery promptly.

### Performing a Flip
- To perform an automatic flip, maintain a steady hover and push down the right stick.

### Acro Mode
- Press the right button on the front of ATOMJoy to switch to Acro Mode. Press again to return to Stabilize Mode. You can check the mode on the ATOMJoy LCD screen.
- Acro Mode does not automatically maintain posture. You need to manually control the posture.
- The drone rotates at a speed proportional to the deflection of the right stick. Returning the stick to neutral stops the rotation. However, the tilt remains and must be corrected manually.

### Altitude Hold Mode (Under Development)
- StampFly is always under development, and altitude control is not yet complete.
- When landed, press the left button on the front of ATOMJoy once to enter Altitude Hold Mode.
- Initially, altitude control is manual. As you slowly increase altitude, the tail LED turns pink, indicating Altitude Hold Mode.
- Once in Altitude Hold Mode, return the left stick to neutral.
- In neutral, the left stick can be used to gradually change altitude.
- Altitude Hold Mode changes altitude slowly and cannot make rapid altitude changes.
- Changes in the height of the ground will cause the drone to adjust altitude accordingly.
- Sudden altitude changes can cause instability or a reset, resulting in a crash.
- For landing, gradually lower the altitude and, when close to the ground, press the left stick to disarm.

### Mode 3
- The developer's favorite mode.
- Press and hold the left button on the front of ATOMJoy while turning it on to enter Mode 3. This mode cannot be changed until the switch is turned off.
- In Mode 3, the left and right stick controls of Mode 2 are reversed. However, the functions of the front buttons do not change.

### Impact Stop Function
- The motors automatically stop when a significant impact is detected.
