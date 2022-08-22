# maptool-new-makedeb-deployqt

## 说明
  新maptool打包工具;

## 使用方法
  1. 打包指令：./make-deb-deployqt <包名> <版本号> <更新信息>  
  2. 相关参数说明： <包名>: 一般为 --- maptool-new  
		   <版本号>: 根据版本信息叠加 如:1.0.3  
		   <更新信息>: 根据更新可随意填写  
```
  ./make-deb-deployqt maptool-new 1.0.3 update
```

## 安装包安装工具
  1. 终端输入指令：git clone https://gitee.com/ctinav/cti_maptool.git && cd cti_maptool
  2. 选择版本进行安装：sudo dpkg -i *.deb
  3. 启动工具：终端输入 maptool-run
