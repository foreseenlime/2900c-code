using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor X1R;
extern motor X2R;
extern motor X1L;
extern motor X2L;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );