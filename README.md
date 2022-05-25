# 拘束条件付きICP

## テスト

~~~
python test.py
~~~

## 説明  
cicp_solverは指定した座標軸のみを移動させるICP処理を行います。このテストではZ平行移動とZ回転のみでのベストフィットを求めています。  
正解はベース座標系にて、Z移動0mm、Z回転3°、しています。  
これに対してソルバーは、Z移動0.0086mm、Z回転2.88°、と推定します。

## ファイル

|name|description|
|:---|:---|
|data/surface_1.ply|マスター点群データ。コード中では"source"とラベリング|
|data/test_RZ3_1.ply|シーン点群データ。コード中では"target"とラベリング|
|data/camera_master0.yaml|点群データが存在する座標系|
|cicp_solver.py|ソルバー。他のソルバーと同様、learnメソッドにてマスターを与え、solveメソッドにてシーンを与えます|
|tflib.py|座標変換表記を変換するライブラリ。rovi_utilsの同名ファイルのコピー|

