# PWM输出通道定义有问题，飞腾派使用的是通道1，但是把通道定义死为0，导致设置死区后输出的脉冲为周期-设置的脉冲
## 查看PWM设置的源代码，发现设置时调用的宏定义PWM_CHANNEL_USE，但是这个宏定义设置为0
![1](/images/PWM_issue_1.png)
![2](/images/PWM_issue_2.png)
## 修改PWM_CHANNEL_USE的宏定义，添加条件如果是飞腾派则设置为1
![3](/images/PWM_issue_3.png)