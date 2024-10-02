# Carck_ros2

このプロジェクトは、ROS2を使用して画像を取得し、画像処理を行うためのパイプラインを提供します。画像を取得するためのパブリッシャーと、処理された画像を受信するためのサブスクライバーが含まれています。

## 必要条件

- ROS2（Foxyまたはそれ以降のバージョン）
- OpenCV
- cv_bridge

## セットアップ

プロジェクトをビルドするには、次のコマンドを実行してください。

```bash
colcon build
source install/setup.bash
```

## 実行

画像をパブリッシュするノードの実行
```bash
ros2 run crack image_publisher
```

画像を受信して処理するノードを実行
```bash
 ros2 run crack image_subscriber
```

## 操作方法
1.PublisherとSubscriberの両方を起動すると"Input_Images"に含まれている画像を読み込まれ画像が表示されます  
2.アルミの板の角4点を左クリックし、何らかのキーを押します  
3.射影変換された画像をトラックバーを操作し、クラックが検出されるよう画像処理をしてください  
4."c"ボタンをクリックすることで処理が終わり、"Output_Images"に画像が保存されます  

## 注意事項
・フォルダのInput_Image、Output_Imageのパスはdetect.py、image_publisher.pyのコードから変えることができます。

