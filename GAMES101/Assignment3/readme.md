## 101作业3  
终于到了光栅化部分的最后一次作业 这次作业的内容丰富不少 可以整的幺蛾子也不少  
### 必需内容 略  
### 提高内容 略  
### 个人尝试  
#### 移植上次作业中的内容  
##### 基本移植  
我们仍然按照作业2中的思路将自由视角以及一系列对框架的修复移动过来 真让我没想到的是原来的这个框架藏起来的bug真不少  
首先将上一次作业的内容都搬过来 如果正常实现的话 自由视角还是很舒适的  
然而发现了一个有点搞笑的bug 把视角往前到靠近小牛时会直接崩溃 最后发现竟然是set_pixel函数里少减了一个1 更搞笑的是作业2中的这部分实现都是正确的（助教要不要看看自己在写什么）  
此外在实验的时候发现移近视角会变得非常卡 不过这个是正常现象 回忆一下在三角形光栅化的过程中我们只判断三角形对应包围盒内的像素 所以当视角移近时 三角形对应的包围盒内的像素数会变得非常多 从而导致光栅化的过程要算的更多了  
##### 更多修复  
由于本次作业框架的复杂性还有更多的问题有待解决  
首先是phong光照模型 我发现phong模型在框架中的实现和我们的可变相机并没有适配 eye_pos竟然偷偷定义了一份 和我在全局中定义的不一样 这里需要对光照模型进行修复 正确的话应该在移动的过程中看到高光等光强的变化 当然使用其他渲染模式的时候也是同理  
其次就是我还希望做进一步的优化 由于贴近的情况下即使是O3优化也顶不住 所以进行了如下优化  
* 背面剔除（backface culling）：这个思想是我在虎书中看到的 由于背向的三角形不会被渲染 从而可以提前排除 以我在本机上生成的火焰图为例 优化前的rasterize_triangle函数占用了92%左右的时间 优化后降低至89%  
  （我还以为这个剔除的效果会很显著 但很遗憾看来只有一点点提升）  
* 求交优化：之前判断一个点是否在三角形内用的是insideTriangle 但是我发现这个函数比直接算重心坐标还慢。。 于是直接求重心坐标然后判断三个坐标是否合规即可 优化后降低至86%  
* interpolate优化：首先我简单的移除了interpolate函数中的weight参数因为它一直被设定为1完全没有真正使用过 然后我将interpolate函数设为了inline以减少调用开销  
  又一次让我感到诧异的是 二维的interpolate函数明明也可以写成上面的三维一样的形式进而使用inline加速 为什么要写成有点复杂的形式 改了改了 优化后降低至82%  

贪性能贪了半天 得到了差不多10%的提升 怎么说呢 确实有点少 但是聊胜于无吧  
这一番操作加上O3优化 在我的虚拟机上终于是几乎不卡了 只有长按前进穿过小牛的时候还会卡顿一下   
***
#### Line Drawing & Cool-to-Warm Shading  
虎书中提及了给艺术家使用的shading方式（书中10.3一节） 这里就尝试实现一下  
这两种渲染方式最终应该一起使用 先一个一个看  
##### Cool-to-Warm Shading  
Cool-to-Warm shading的逻辑很简单 首先定义一个cool color和一个warm color 然后设置一个warm light direction 最后将颜色渲染为cool color和warm color的差值  
具体而言 我们计算两个颜色的比例为$k_w = \frac{1+n\cdot l}{2}$ n为法向量 l为上面定义的暖色光方向  
这一渲染思路还可以跟phong模型结合起来 注意原本实现phone模型时采用的是物体自带的color作为diffuse光照的颜色参数$k_d$ 只要替换为我们上面得到的颜色值即可  
话虽如此 如果试图在shader中进行实现的话就会遇到一个问题 我们拿到的法向量已经是经过变换的 也即在相机坐标系下 这就导致我们得到的结果会随着视角变动而变动  
todo
##### Line Drawing  
线框渲染的目的是画出模型在视角上的边缘 最终期望看到的效果是一个黑色的轮廓线  
我们如何找到这些边缘呢 考虑任意两个共用边的三角形 我们的逻辑是这两个三角形的法向量分别和视角方向点乘 如果得到的两个结果符号不同 则说明恰好一个三角形仍然偏向正向 一个已经转向背向 从而这个边就是我们要找的边  
待续实现  