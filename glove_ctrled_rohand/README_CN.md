# 手套控制 ROHand

## 准备

安装python和pip
进入命令环境，如windows下的command或者linux下的BASH
进入演示项目目录，例如：

```SHELL
cd glove_ctrled_rohand
```

安装依赖的python库：

```SHELL
pip install -r requirements.txt
```

---

打开`glove_ctrled_hand.py` 并修改设备地址，例如：

```python
NODE_ID = 2
```

---

根据手套类型选择PosInput类型，下面两种类型中只能选择一种：

* 使用蓝牙版手套:

```python
from pos_input_ble_glove import PosInputBleGlove as PosInput
```

* 使用USB版手套:

```python
from pos_input_usb_glove import PosInputUsbGlove as PosInput
```

## 运行

```python
python glove_ctrled_hand.py
```

按照指示进行初始标定后，即可通过手套控制灵巧手。
