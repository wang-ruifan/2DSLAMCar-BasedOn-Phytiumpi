/******************************************************************************
 *
 *  file: mainwindow.cpp
 *  author: Ruifan Wang
 *  description: This file is the mainwindow file of the car_gui node. It uses
 *              the mainwindow class to create the GUI of the car control system.
 *
*******************************************************************************/
#include "mainwindow.h"
#include "rpmsg.h"

/******************************************************************************
 *
 *  name: MainWindow
 *  class: MainWindow
 *  description: This is the constructor of the MainWindow class. It initializes
 *              the GUI of the car control system, including the error_label, rocker
 *              , the speed display lcd, the speed control combo, the car display
 *              and the start button.
 *
*******************************************************************************/
MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
{
	this->setGeometry(0, 0, 400, 1080);

	labelInit();                //提示条初始化
	buttonInit();               //按钮初始化
	lcdInit();                  //车速显示屏初始化
	rockerInit();               //摇杆初始化
	comboInit();                //速度选择框初始化
	pixmapInit();               //图片初始化
}

/******************************************************************************
 *
 *  name: ~MainWindow
 *  class: MainWindow
 *  description: This is the destructor of the MainWindow class.
*******************************************************************************/
MainWindow::~MainWindow()
{
}

/******************************************************************************
 *
 *  name: labelInit
 *  class: MainWindow
 *  description: This function initializes the error_label, the vx_label, the vz_label.
 *
*******************************************************************************/
void MainWindow::labelInit()
{
	labelError = new QLabel("等待启动...", this);
	labelError->setGeometry(0, 25, 400, 50);
	labelError->setAlignment(Qt::AlignCenter);

	labelVx = new QLabel("前进速度", this);
    labelVx->setGeometry(50, 485, 100, 50);
	labelVx->setAlignment(Qt::AlignCenter);

	labelVz = new QLabel("转向速度", this);
    labelVz->setGeometry(250, 485, 100, 50);
	labelVz->setAlignment(Qt::AlignCenter);
}

/******************************************************************************
 *
 *  name: buttonInit
 *  class: MainWindow
 *  description: This function initializes the start button.
 *
*******************************************************************************/
void MainWindow::buttonInit()
{
	btnStart = new QPushButton("启动", this);
    btnSave = new QPushButton("保存地图", this);

    btnStart->setGeometry(100, 950, 80, 40);
    btnSave->setGeometry(220, 950, 80, 40);

	connect(btnStart, SIGNAL(clicked()), this, SLOT(pushButtonStart_Clicked()));

    btnSave->setEnabled(false);
}

/******************************************************************************
 *
 *  name: pushButtonStart_Clicked
 *  class: MainWindow
 *  description: This function is the slot function of the start button. It will
 *              start the slave core and slave core programe using OpenAMP. It will
 *              also start the rpmsg communication between the master core and the
 *              slave core. If the start process is successful, the start button will
 *              change to the end button and the startFlag will be set to true. If the
 *              start process is failed, the error message will be displayed on the error_label.
 *
*******************************************************************************/
void MainWindow::pushButtonStart_Clicked()
{
	btnStart->setEnabled(false);
	bool errorFlag = true;
	labelError->setText("正在启动从核及底盘控制...");
	delay_ms(2000);
	openampInit();            //OpenAMP启动函数
	int ret = rpmsgInit();    //rpmsg初始化函数
	switch (ret) {
	case -1:
		qDebug() << "open rpmsg_ctrl0 failed";
		labelError->setText("rpmsg控制驱动打开失败，请以root用户启动或重启后重试");
		errorFlag = false;
		break;
	case 0:
		qDebug() << "toctl rpmsg creat ept failed";
		labelError->setText("rpmsg节点创建失败，请以root用户启动或重启后重试");
		errorFlag = false;
		break;
	case 1:
		qDebug() << "open rpmsg0 failed";
		labelError->setText("rpmsg通信驱动打开失败，请以root用户启动或重启后重试");
		errorFlag = false;
		break;
	case 2: qDebug() << "rpmsg Init success"; break;
	}
	if (errorFlag) {
		qDebug() << "start completed";
		labelError->setText("启动成功！可以开始控制");
		startFlag = true;
	}
	disconnect(btnStart, SIGNAL(clicked()), this, SLOT(pushButtonStart_Clicked()));
	btnStart->setText("停止");
	btnStart->setEnabled(true);
	connect(btnStart, SIGNAL(clicked()), this, SLOT(pushButtonEnd_Clicked()));
    btnSave->setEnabled(true);
    connect(btnSave, SIGNAL(clicked()), this, SLOT(pushButtonSave_Clicked()));
}

/******************************************************************************
 *
 *  name: pushButtonEnd_Clicked
 *  class: MainWindow
 *  description: This function is the slot function of the end button. It will
 *              stop the rpmsg communication between the master core and the
 *              slave core. It will also close the mainwindow.
 *
*******************************************************************************/
void MainWindow::pushButtonEnd_Clicked()
{
	rpmsgQuit();
	this->close();
}


/******************************************************************************
 *
 *  name: pushButtonSave_Clicked
 *  class: MainWindow
 *  description: This function is the slot function of the save button. It will
 *              save the current map to car_ws/map.
 *
*******************************************************************************/
void MainWindow::pushButtonSave_Clicked()
{
    system("cd /home/user/car_ws");
    system("source install/setup.bash");
    system("ros2 run nav2_map_server map_saver_cli -t map -f car_map");
    labelError->setText("保存成功！已保存到car_ws目录下！");
}

/******************************************************************************
 *
 *  name: rockerInit
 *  class: MainWindow
 *  description: This function initializes the rocker center point position.
 *
*******************************************************************************/
void MainWindow::rockerInit()
{
	SmallCir_xy.setX(200);
	SmallCir_xy.setY(825);

	BigCir_xy = SmallCir_xy;

	MousePressFlag = false;
}

/******************************************************************************
 *
 *  name: lcdInit
 *  class: MainWindow
 *  description: This function initializes the speed display lcd.
 *
*******************************************************************************/
void MainWindow::lcdInit()
{
	lcdDisplayVx = new QLCDNumber(this);
	lcdDisplayVz = new QLCDNumber(this);

    lcdDisplayVx->setGeometry(25, 525, 150, 75);
    lcdDisplayVz->setGeometry(225, 525, 150, 75);

    lcdDisplayVx->setDigitCount(4);
    lcdDisplayVz->setDigitCount(4);

	lcdDisplayVx->setSegmentStyle(QLCDNumber::Flat);
	lcdDisplayVz->setSegmentStyle(QLCDNumber::Flat);

    lcdDisplayVx->setSmallDecimalPoint(true);
    lcdDisplayVz->setSmallDecimalPoint(true);

	lcdDisplayVx->display("0.00");
	lcdDisplayVz->display("0.00");
}

/******************************************************************************
 *
 *  name: comboInit
 *  class: MainWindow
 *  description: This function initializes the max speed control combo.
 *
*******************************************************************************/
void MainWindow::comboInit()
{
	speedBox = new QComboBox(this);

	speedBox->setGeometry(138, 650, 124, 30);

	speedBox->addItem("低速（默认）");
	speedBox->addItem("中速");
	speedBox->addItem("高速");
	speedBox->addItem("极速（不建议）");

	speedBox->setCurrentIndex(0);
}

/******************************************************************************
 *
 *  name: pixmapInit
 *  class: MainWindow
 *  description: This function initializes the car and the rocker pixmap.
 *
*******************************************************************************/
void MainWindow::pixmapInit()
{
	car_Pixmap.load("/home/user/car_ws/src/car_gui/resource/car.png");
	bigCircle_Pixmap.load("/home/user/car_ws/src/car_gui/resource/rocker_max.png");
	smallCircle_Pixmap.load("/home/user/car_ws/src/car_gui/resource/rocker_min.png");
}

/******************************************************************************
 *
 *  name: delay_ms
 *  class: MainWindow
 *  description: This function is a non-blocking delay function.
 *
*******************************************************************************/
void MainWindow::delay_ms(int ms)
{
	QTime t = QTime::currentTime().addMSecs(ms);
	while (QTime::currentTime() < t)
		QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

/******************************************************************************
 *
 *  name: openampInit
 *  class: MainWindow
 *  description: This function starts the slave core and slave core programe using OpenAMP.
 *
*******************************************************************************/
void MainWindow::openampInit()
{
	system("echo start > /sys/class/remoteproc/remoteproc0/state");
	delay_ms(2000);
	labelError->setText("启动完成！正在启动从核通信...");
	system("echo rpmsg_chrdev > /sys/bus/rpmsg/devices/virtio0.rpmsg-openamp-demo-channel.-1.0/driver_override");
	delay_ms(1000);
	system("modprobe rpmsg_char");
}

/******************************************************************************
 *
 *  name: paintEvent
 *  class: MainWindow
 *  description: This function is the paint event function of the mainwindow. It will
 *              draw draw the rock accoring to the mouse position to let the rocker
 *              follow the mouse and the car according to the angle.
 *
*******************************************************************************/
void MainWindow::paintEvent(QPaintEvent* event)
{
	Q_UNUSED(event);

	QPainter painter(this);

	//antialiasing
	painter.setRenderHint(QPainter::Antialiasing, true);

	painter.setRenderHints(QPainter::SmoothPixmapTransform);

	//draw the big circle of the rocker
	painter.drawPixmap(SmallCir_xy.x() - BIG_CIRCLE_RADIUS, SmallCir_xy.y() - BIG_CIRCLE_RADIUS, \
		BIG_CIRCLE_RADIUS * 2, BIG_CIRCLE_RADIUS * 2, bigCircle_Pixmap);

	//draw the small circle of the rocker
	painter.drawPixmap(BigCir_xy.x() - SMALL_CIRCLE_RADIUS, BigCir_xy.y() - SMALL_CIRCLE_RADIUS, \
		SMALL_CIRCLE_RADIUS * 2, SMALL_CIRCLE_RADIUS * 2, smallCircle_Pixmap);

	//draw the car
	painter.translate(200, 278);
	painter.rotate(angle);
	painter.translate(-200, -278);
	painter.drawPixmap(72, 150, 256, 256, car_Pixmap);
}

/******************************************************************************
 *
 *  name: mouseMoveEvent
 *  class: MainWindow
 *  description: This function is the mouse move event function of the mainwindow. It will
 *              get the position of the mouse and then calculate the x and y of the rocker.
 *
*******************************************************************************/
void MainWindow::mouseMoveEvent(QMouseEvent* e)
{
	QPoint rocker_xy;
	QByteArray xy;
	xy.resize(2);
	//Get the current mouse position
	rocker_xy = e->pos();

	//If the mouse is pressed inside the big circle, will calculate the position of the rocker
	if (MousePressFlag)
	{
		//If the small circle is out of the big circle, the rocker will be at the edge of the big circle
		if (pow((rocker_xy.x() - SmallCir_xy.x()), 2) + pow((rocker_xy.y() - SmallCir_xy.y()), 2) > 8100)
		{
			//Calculate the x and y of the rocker
			x = int(90 * cos(atan2(abs(rocker_xy.y() - SmallCir_xy.y()), abs(rocker_xy.x() - SmallCir_xy.x()))));
			y = int(90 * sin(atan2(abs(rocker_xy.y() - SmallCir_xy.y()), abs(rocker_xy.x() - SmallCir_xy.x()))));

			//Calculate the position of the rocker according to the position of the mouse
			if (rocker_xy.x() > SmallCir_xy.x() && rocker_xy.y() > SmallCir_xy.y())
			{
				//The first quadrant
				BigCir_xy.setX(x + SmallCir_xy.x());
				BigCir_xy.setY(y + SmallCir_xy.y());
			}
			else if (rocker_xy.x() < SmallCir_xy.x() && rocker_xy.y() > SmallCir_xy.y())
			{
				//The second quadrant
				BigCir_xy.setX(-x + SmallCir_xy.x());
				BigCir_xy.setY(y + SmallCir_xy.y());
				x = -x;
			}
			else if (rocker_xy.x() < SmallCir_xy.x() && rocker_xy.y() < SmallCir_xy.y())
			{
				//The third quadrant
				BigCir_xy.setX(-x + SmallCir_xy.x());
				BigCir_xy.setY(-y + SmallCir_xy.y());
				x = -x;
				y = -y;
			}
			else if (rocker_xy.x() > SmallCir_xy.x() && rocker_xy.y() < SmallCir_xy.y())
			{
				//The fourth quadrant
				BigCir_xy.setX(x + SmallCir_xy.x());
				BigCir_xy.setY(-y + SmallCir_xy.y());
				y = -y;
			}
		}
		else//The rocker is inside the big circle and the position of the rocker is the same as the mouse
		{
			BigCir_xy = rocker_xy;
			x = rocker_xy.x() - SmallCir_xy.x();
			y = rocker_xy.y() - SmallCir_xy.y();
		}
		xy[0] = char(x);
		xy[1] = char(y);
		//qDebug() << x << y;
		//update();
	}

	MapRemov_Old = rocker_xy;
}

/******************************************************************************
 *
 *  name: mouseReleaseEvent
 *  class: MainWindow
 *  description: This function is the mouse release event function of the mainwindow. It will
 *              set the MousePressFlag to false and the rocker will return to the center of the big circle.
 *
*******************************************************************************/
void MainWindow::mouseReleaseEvent(QMouseEvent* e)
{
	Q_UNUSED(e);

	//The mouse is released, reset the MousePressFlag to false
	MousePressFlag = false;

	//The rocker will return to the center of the big circle
	BigCir_xy.setX(SmallCir_xy.x());
	BigCir_xy.setY(SmallCir_xy.y());

	x = 0;
	y = 0;
}

/******************************************************************************
 *
 *  name: mousePressEvent
 *  class: MainWindow
 *  description: This function is the mouse press event function of the mainwindow. It will
 *              set the MousePressFlag to true and get the current position of the mouse.
 *
*******************************************************************************/
void MainWindow::mousePressEvent(QMouseEvent* e)
{
	QPoint rocker_xy;

	//Get the current mouse position
	rocker_xy = e->pos();
	//qDebug() << "rocker position:" << rocker_xy;

	//If the mouse is pressed inside the big circle, will set the MousePressFlag to true
	if (pow((rocker_xy.x() - SmallCir_xy.x()), 2) + pow((rocker_xy.y() - SmallCir_xy.y()), 2) <= 8100)
	{
		MousePressFlag = true;
		//If the rpmsg communication is started, and mouse is pressed to control the rock
		//will set the control flag to true
		if (startFlag) {
			controlFlag = true;
		}
	}
	else
	{
		MapRemov_Old = rocker_xy;
	}
}

/******************************************************************************
 *
 *  name: controlCar
 *  class: MainWindow
 *  description: This function is the control function of the car. It will get the current
 *              speed and the break flag of the car from the slave core. It will also get the
 *              target speed of the car according to the position of the rocker. Then it will send
 *              the target speed to the slave core to control the car. If the break flag is 1, which
 *              means there is an obstacle in front of the car, the error message will be displayed
 *              on the error_label.
 *
*******************************************************************************/
void MainWindow::controlCar(int* flag, float* vx, float* vz)
{
	//Initialize the speed and the break flag
	int break_flag = 0;
	float current_vx = 0.0, current_vz = 0.0, target_vx = 0.0, target_vz = 0.0, r = 0.0;
	//If the car is started and the control is enabled, will get the current speed and the break flag of the car
	if (startFlag && controlFlag) {
		//Get the max speed according to the speed control combo
		int index = speedBox->currentIndex();
		switch (index) {
		case 0: maxSpeed = 0.1; break;
		case 1: maxSpeed = 0.2; break;
		case 2: maxSpeed = 0.3; break;
		case 3: maxSpeed = 0.5; break;
		}

		//Get the current speed and the break flag of the car using rpmsgGetCarData
		rpmsgGetCarData(&break_flag, &current_vx, &current_vz);

		//Assign the current speed and the break flag to the flag and the vx and vz
		//Transfer these data back to car_gui node
		*flag = break_flag;
		*vx = current_vx;
		*vz = current_vz;

		//Display the current speed on the lcd display
        lcdDisplayVx->setSmallDecimalPoint(true);
        lcdDisplayVz->setSmallDecimalPoint(true);
        QString stringVx = QString::number(current_vx, 'f', 2);
        QString stringVz = QString::number(current_vz, 'f', 2);
        lcdDisplayVx->display(stringVx);
        lcdDisplayVz->display(stringVz);

		//Calculate the target speed according to the position of the rocker
		target_vx = -1.0 * maxSpeed * y / 90.0;
		target_vz = -1.0 * maxSpeed * x / 90.0;
		//if the target_vx is negative, the target_vz will be negative
		if (target_vx < 0) {
			target_vz = target_vz * -1.0;
		}
		r = target_vx / target_vz;

		if (break_flag == 1) {
			//If the break flag is 1, which means there is an obstacle in front of the car, the error message will be displayed
			labelError->setText("前方有障碍物，紧急制动触发！");
		}
		else if ((r < 0.35 && r > 0) || (r < 0 && r > -0.35)) {
			//If the angle is too large, the error message will be displayed
			labelError->setText("转向角度超过限幅！");
		}
		else {
			//If the control is normal, the error message will be cleared
			labelError->setText("");
		}

		//Send the target speed to the slave core using rpmsgSendCarData
		rpmsgSendCarData(target_vx, target_vz);
	}
	else {
		//If the car is not started or the control is not enabled

		//The break flag will be set to 1
		break_flag = 1;
		//The current speed will be set to 0
		current_vx = 0;
		current_vz = 0;
		//Assign the current speed and the break flag to the flag and the vx and vz
		*flag = break_flag;
		*vx = current_vx;
		*vz = current_vz;
	}

	//Update the car display angle according to the current speed
	angle = 0.0;
	if (current_vx > 0.01) {
		int Vx = 100 * current_vx;
		int Vz = 100 * current_vz;
		angle = atan(-1 * Vz / Vx) * (180.0 / 3.1415926) * 1.0;
	}
	else if (current_vx < -0.01) {
		int Vx = 100 * current_vx;
		int Vz = 100 * current_vz;
		angle = atan(Vz / (-1 * Vx)) * (180.0 / 3.1415926) * 1.0;
	}

	//Update the window display
	update();
}
