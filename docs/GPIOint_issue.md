# GPIO中断注册函数有问题，传入的变量不调用，直接设置为高电平触发和单次触发
## 查看FreeRTOSSDK中设置中断类型的源代码，发现情况不对，没有调用裸机SDK中设置GPIO中断的函数
![1](/images/GPIOint_issue_1.png)
## 修改FreeRTOSSDK中设置中断类型的源代码，读取config中的irq_type中断类型变量，并传入裸机SDK中设置GPIO中断的函数
![2](/images/GPIOint_issue_2.png)
![3](/images/GPIOint_issue_3.png)
![4](/images/GPIOint_issue_4.png)
## 具体发现过程和修改
![5](/images/GPIOint_issue_5.png)
![6](/images/GPIOint_issue_6.png)
![7](/images/GPIOint_issue_7.png)
![8](/images/GPIOint_issue_8.png)
![9](/images/GPIOint_issue_9.png)
![10](/images/GPIOint_issue_10.png)
# 好像还有控制器中断和pin中断配置的问题，但是找不到聊天记录了