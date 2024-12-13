#define NUM_INPUT_MFS 7
#define NUM_KP_MFS 7
#define NUM_KI_MFS 7
#define NUM_RULES 49

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6

#define MIN_Kp -50
#define MAX_Kp 50
#define Kp_STEP (MAX_Kp - MIN_Kp) / (NUM_KP_MFS - 1)
#define MIN_Ki -5
#define MAX_Ki 5
#define Ki_STEP (MAX_Ki - MIN_Ki) / (NUM_KI_MFS - 1)

// 定义三角形隶属度函数结构
typedef struct {
	float a, b, c;  // 三角形的三个点
} MembershipFunction;

// 定义模糊变量结构
typedef struct {
	MembershipFunction mfs[7];  // 隶属度函数
	float value;				// 输入值用于模糊化
} FuzzyVariable;

// 定义模糊规则结构
typedef struct {
	int inputEIndex;	// 输入E MF的索引
	int inputEcIndex;	// 输入Ec MF的索引
	int outputKpIndex;  // 输出Kp MF的索引
	int outputKiIndex;  // 输出Ki MF的索引
} FuzzyRule;

// 模糊控制器结构
typedef struct {
	FuzzyVariable inputE;
	FuzzyVariable inputEc;
	FuzzyVariable outputKp;
	FuzzyVariable outputKi;
	FuzzyRule rules[NUM_RULES];
} FuzzyController;

void InitFuzzyMotorPIController();
void EvalFuzzyMotorPIController(float e, float ec, float* ki, float* kp);
