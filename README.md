# HBUT2025视觉代码

## 编译命令

```shell
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-humble-asio-cmake-module
./build.sh
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ros2 launch bringup armor_launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 param load /armor_detector src/bringup/config/params.yaml
```
### 启动相机节点
  //相机节点
  ros2 launch mindvision_camera mv_launch.py
  //相机标定
	ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.02000  image:=/image_raw


## 编码规范：

### 头文件
#### #define 保护
```c++
#ifndef FOO_BAR_BAZ_H_
#define FOO_BAR_BAZ_H_
...
#endif // FOO_BAR_BAZ_H_
```
#### #include 的路径及顺序
使用标准的头文件包含顺序可增强可读性, 避免隐藏依赖: 相关头文件, C 库, C++ 库, 其他库的 .h, 本项目内的 .h.

又如, dir/foo.cc 或 dir/foo_test.cc 的主要作用是实现或测试 dir2/foo2.h 的功能, foo.cc 中包含头文件的次序如下:
   1. dir2/foo2.h (优先位置, 详情如下)
   2. C 系统文件
   3. C++ 系统文件
   4. 其他库的 .h 文件
   5. 本项目内 .h 文件

### 命名
#### 文件名
文件名要全部小写, 可以包含下划线 (\_) 或连字符 (-), 依照项目的约定. 如果没有约定, 那么 “_” 更好.
```txt
my_useful_class.cc
my-useful-class.cc
myusefulclass.cc
```
C++ 文件要以 .cc 结尾, 头文件以 .h 结尾. 专门插入文本的文件则以 .inc 结尾

#### 类型命名
类型名称的每个单词首字母均大写, 不包含下划线: `MyExcitingClass`, `MyExcitingEnum`  


所有类型命名 —— 类, 结构体, 类型定义 (typedef), 枚举, 类型模板参数 —— 均使用相同约定, 即以大写字母开始, 每个单词首字母均大写, 不包含下划线. 例如:
```c++
// 类和结构体
class UrlTable { ...
class UrlTableTester { ...
struct UrlTableProperties { ...

// 类型定义
typedef hash_map<UrlTableProperties *, string> PropertiesMap;

// using 别名
using PropertiesMap = hash_map<UrlTableProperties *, string>;

// 枚举
enum UrlTableErrors { ...
```

#### 变量命名
变量 (包括函数参数) 和数据成员名一律小写, 单词之间用下划线连接. 类的成员变量以下划线结尾, 但结构体的就不用, 如: `a_local_variable`, `a_struct_data_member`, `a_class_data_member_`.

##### 普通变量命名
```c++
string table_name;  // 好 - 用下划线.
string tablename;   // 好 - 全小写.

string tableName;  // 差 - 混合大小写
```

##### 类数据成员
不管是静态的还是非静态的, 类数据成员都可以和普通变量一样, 但要接下划线.

```c++
class TableInfo {
  ...
 private:
  string table_name_;  // 好 - 后加下划线.
  string tablename_;   // 好.
  static Pool<TableInfo>* pool_;  // 好.
};
```

##### 结构体变量
不管是静态的还是非静态的, 结构体数据成员都可以和普通变量一样, 不用像类那样接下划线:

```c++
struct UrlTableProperties {
  string name;
  int num_entries;
  static Pool<UrlTableProperties>* pool;
};
```

##### 函数命名

常规函数使用大小写混合, 取值和设值函数则要求与变量名匹配: `MyExcitingFunction()`, `MyExcitingMethod()`, `my_exciting_member_variable()`, `set_my_exciting_member_variable()`

##### 枚举命名
枚举的命名应当和 常量 或 宏 一致: kEnumName 或是 ENUM_NAME.

##### 宏命名
像这样命名: MY_MACRO_THAT_SCARES_SMALL_CHILDREN.