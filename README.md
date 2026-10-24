1、电脑上根据自己电脑型号要安装PC-Service
https://github.com/XR-Robotics/XRoboToolkit-PC-Service

Jaka主机是ARM64架构，必须安装该Release中的arm64安装包：
XRoboToolkit-PC-Service_1.0.0.0_arm64.deb
amd64/x64安装包不能用于Jaka主机。

2、查看jaka主机IP（用于quest3上面的apk输入）
hostname -I

3、在quest3上面运行XRoboToolKit的app：
adb shell monkey -p com.xrobotoolkit.client.quest -c android.intent.category.LAUNCHER 1

4、quest3的MAC地址：
50:99:03:0a:38:78
