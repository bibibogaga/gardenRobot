# only_livox_my
this is patch witch livox_relocalization&amp;livox_mapping,for livox mid-70

modify

add ./sh
modify launch:

<<<<<<< HEAD
=======


## 脚本说明：

1.setup_livox.sh

启动livox_mid-70的sdk，主要用于与电脑端通信可视化，查看它是否连接

2.setup_rosdriver.sh

启动livox的ros driver，若想运行建图程序，必须先运行这个来启动livox

3.需要关闭livox_ros_driver

```bash
cd /home/yoga/my_work/livox_formid-40/src/Livox-SDK-master/build/sample/lidar_lvx_file
./lidar_lvx_sample
```

4.changebag.sh

将lvx文件转换成bag包

5.automapping.sh

建图

6.autolocal.sh

重定位
>>>>>>> 2
