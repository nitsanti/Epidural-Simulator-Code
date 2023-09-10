//==============================================================================
/*
	Software License Agreement (BSD License)
	Copyright (c) 2003-2016, CHAI3D.
	(www.chai3d.org)

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	* Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.

	* Redistributions in binary form must reproduce the above
	copyright notice, this list of conditions and the following
	disclaimer in the documentation and/or other materials provided
	with the distribution.

	* Neither the name of CHAI3D nor the names of its contributors may
	be used to endorse or promote products derived from this software
	without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.

	\author    <http://www.chai3d.org>
	\author    Francois Conti
	\version   3.2.0 $Rev: 1907 $
	*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "GEL3D.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include "CCollisionAABB.h"
#include "CMesh.h"
//Nitsan: include all the h files that we added
#include "Timer.h"
//#include "Protocol.h"
//#include "Haptic.h"
#include "FileHandle.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>

//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
	C_STEREO_DISABLED:            Stereo is disabled
	C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
	C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
	C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
	*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

bool two_devices = true; //Nitsan: only when we choose to use both devices

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;
//cHapticDeviceHandler* handler2; //Nitsan: trying to add the 2nd haptic device


// a haptic device 
cGenericHapticDevicePtr hapticDevice;
cGenericHapticDevicePtr hapticDevice2; //Nitsan: trying to add the 2nd haptic device

// force scale factor
double deviceForceScale;

// scale factor between the device workspace and cursor workspace
double workspaceScaleFactor;
double workspaceScaleFactor2; //Nitsan: haptic device 2

// desired workspace radius of the virtual cursor
double cursorWorkspaceRadius;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;

// Nitsan: I added this
// a virtual tool representing the haptic device in the scene 
cToolCursor* tool;

// Nitsan: I added this
// virtual drill mesh
cMultiMesh* drill;

//Nitsan: I added this
//This is a temp to convert pos.z() to mm
double posZ;
double posX_fixed; //Nitsan: we fix this according to the region that we're in 
double scaling; //Nitsan: this is to scale the forces so that we will not get saturation of the robot
double posX; //Nitsan: we need this and posY for the bone hitting
double posY; //Nitsan: I added this

// Nitsan: to indicate if we are in ES
double inES = -1; //Nitsan: -1 means before (failed epidural), 0 means inside ES (success) and 1 means after (dural puncture)
bool dural_puncture = false; //Nitsan: once the dura has been punctured, it cannot be un-punctured


bool force_in_2 = false; //Nitsan: if the user hits [s] then it means he wants to start giving forces in the 2nd robot as well
bool injection = false; //Nitsan: this indicates if the trainee has pressed 'D', and injected the medicine

bool startForce2 = false; // Nitsan: indicates if the person is holding the 2nd device
bool firstTime = true; //Nitsan: if this is the first trial we need less k,b
bool print_force2 = true; //Nitsan: if this is true - the printing that indicates the beginning of force in robot 2 will appear
bool print_exit_force2 = true; //Nitsan: you can exit by hitting [N]

//---------------------------------------------------------------------------
// GEL
//---------------------------------------------------------------------------

// deformable world
cGELWorld* defWorld;

// object mesh
cGELMesh* defObject;

// object mesh //Nitsan: adding another box
cGELMesh* defObject2;

// dynamic nodes
cGELSkeletonNode* nodes[10][10]; //Nitsan: I changed this from [10][10]

// Nitsan: linear spring for stiffness to 2nd layer
//cGELLinearSpring* spring2;

// haptic device model //Nitsan: I changed from sphere to cylinder
//cShapeSphere* device;
cShapeCylinder* device;
double deviceRadius;
double deviceHeight; //Nitsan: I added this
cShapeSphere* sphere; //Nitsan: I added this 
double sphere_radius; //Nitsan: I added this

// radius of the dynamic model sphere (GEM)
double radius;

// stiffness properties between the haptic device tool and the model (GEM)
double stiffness;
double stiffness2; //Nitsan: I added this 

//Nitsan: scope to show force
cScope* scope; //Nitsan: I added this

// Nitsan: Thickness of layer 1
cVector3d layer1_end; //Nitsan: I added this
double zlayer1_thickness; //Nitsan: I added this

// Nitsan: Thickness of layer 2
cVector3d layer2_end; //Nitsan: I added this
double zlayer2_thickness; //Nitsan: I added this

//Nitsan: Name of Participant
char Subject_Name[200]; //Nitsan: I added this
double TrialNum;
double NumTrials = 80; // Nitsan: 54 trials + 10 training + 16 generalization

//Nitsan: name of file (string), and file successfully opened or not
char filename[200]; //Nitsan: I added this
FILE* Data_file; //Nitsan: I added this
char filename2[200]; //Nitsan: for robot 2
FILE* Data_file2; //Nitsan: for robot 2
//double counter = 1; //Nitsan: this is just for some checking

//Nitsan: another file for success
char success_filename[200];
FILE* Success_file;

//Nitsan: mark the (0,0) point with a circle
//cLabel* marker; //Nitsan: I added this
//cVector3d enter_point; //Nitsan: this is in order to keep the (x,y) coordinates of the entrance 

// Nitsan: force channels, we need to remember the position in which they left the needle
double x_channel;
double y_channel;
double z_channel;
double k_channel = 500;
double b_channel;
cVector3d velocity;
cVector3d pos_L;
bool create_channel = false;

// Nitsan: Ligamentum Flavum texture:
cMesh* ligamentum;

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// compute forces between tool and environment
cVector3d computeForce(const cVector3d& a_cursor,
	double a_cursorRadius,
	const cVector3d& a_spherePos,
	double a_radius,
	double a_stiffness);

//Nitsan: Decleration of functions:
void OpenDataFile(double);
void OpenSuccessFile(double);
void OpenFolder(void);
bool StartingPosition(double k, double b, bool direction);
void ReadProtocolFile(bool);
double ThicknessLayer[80][9];
bool test;
double thick[9];
void getThickness(void);
void ReadScaling(bool);
double scaling_coef[80];
double weight[80];
void ReadWeight(bool);
void createPertubation(bool firstTimePert);
bool firstTimePert[4];
void StopTrial();
cVector3d createBuldges(double posZ);

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//===========================================================================
/*
	DEMO:    GEM_membrane.cpp

	This application illustrates the use of the GEM libraries to simulate
	deformable object. In this example we load a simple mesh object and
	build a dynamic skeleton composed of volumetric spheres and 3 dimensional
	springs which model torsion, flexion and elongation properties.
	*/
//===========================================================================

int main(int argc, char* argv[])
{
	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------
	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "Epidural Injection Simulator" << endl;
	cout << "Name: "; //Nitsan: I added this
	cin >> Subject_Name; //Nitsan: I added this
	cout << endl; //Nitsan: I added this
	cout << "Trial Num: ";
	cin >> TrialNum;
	while (TrialNum < 1){
		cout << "Trial Num must be 1 or larger. Try again: ";
		cin >> TrialNum;
	}
	cout << endl;
	cout << "Enable Forces in LOR Syringe (Initially): " << endl;
	cout << "[0] No" << endl;
	cout << "[1] Yes" << endl;
	cin >> force_in_2;
	cout << endl; //Nitsan: I added this
	cout << "Test or Control?" << endl;
	cout << "[0] Control" << endl;
	cout << "[1] Test" << endl;
	cin >> test;
	cout << endl;


	cout << "Hi, " << Subject_Name << endl; //Nitsan: I added this
	cout << "Just a moment..." << endl; //Nitsan: I added this
	OpenFolder();
	OpenDataFile(TrialNum); //Nitsan: I added this
	OpenSuccessFile(TrialNum); //Nitsan: openning the success file only once
	ReadProtocolFile(test); //Nitsan: read data from the protocol file
	ReadScaling(test); //Nitsan: read data for scaling the coefficients
	ReadWeight(test); //Nitsan: patient weight
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	// cout << "[s] - Show/Hide GEL Skeleton" << endl;
	cout << "[d] - Identify Epidural space and administer analgesia - and go to next trial" << endl;
	cout << "[s] - Enable forces in 2nd device (connect LOR syringe to needle)" << endl;
	cout << endl;
	cout << "[l] - Leave needle" << endl;
	cout << "[p] - Pick up needle" << endl;
	cout << endl;
	cout << "[m] - Enable/Disable vertical mirroring" << endl;
	cout << "[q] - Exit application" << endl;
	cout << endl << endl;

	// parse first arg to try and locate resources
	resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);


	//-----------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//-----------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "Epidural Simulator Prototype", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

	// initialize GLEW library
#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif


	//-----------------------------------------------------------------------
	// 3D - SCENEGRAPH
	//-----------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	// Nitsan: x=depth into the screen
	// Nitsan: y=left/ right
	// Nitsan: z=up/ down
	camera->set(cVector3d(1.5, 0.0, 1.5),    // camera position (eye) //Nitsan: I changed this from (1.5, 0.0, 1.0)
		cVector3d(0.0, 0.0, 0.0),    // lookat position (target) //Nitsan: I changed this from (0.0, 0.0, 0.0)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector //Nitsan: I changed this from (0.0, 0.0, 1.0)

	// set the near and far clipping planes of the camera
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.02);
	camera->setStereoFocalLength(3.0);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// enable multi-pass rendering to handle transparent objects
	camera->setUseMultipassTransparency(true);

	// create a directional light source
	light = new cDirectionalLight(world);

	// insert light source inside world
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// define direction of light beam
	light->setDir(1.0, 0.0, -1.0); //Nitsan: I changed this from (0,0,-1)


	//-----------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------.

	// desired workspace radius of the cursor
	cursorWorkspaceRadius = 0.7;

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	//if (two_devices)
	//handler2 = new cHapticDeviceHandler(); //Nitsan: haptic device 2
	//handler->update();
	double numDevices = handler->getNumDevices();
	cout << "Number of Devices: " << numDevices << endl;

	/*cHapticDeviceInfo haptic_info;
	if (handler->getDeviceSpecifications(haptic_info, 1))
	cout << "yes" << endl;
	else
	cout << "no" << endl;*/

	// get access to the first available haptic device found
	handler->getDevice(hapticDevice, 0);
	/*if (handler->getDevice(hapticDevice, 0))
		cout << "Success - 1" << endl;*/
	//Nitsan: get access to the second available haptic device found
	if (two_devices) {
		handler->getDevice(hapticDevice2, 1);
		/*if (handler->getDevice(hapticDevice2, 1)) //Nitsan: haptic device 2
			cout << "Success - 2" << endl;*/
	}
	
	cout  << "Trial Num: " << TrialNum << endl;
	// retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
	if (two_devices) {
		cHapticDeviceInfo hapticDeviceInfo2 = hapticDevice2->getSpecifications();
		workspaceScaleFactor2 = cursorWorkspaceRadius / hapticDeviceInfo2.m_workspaceRadius;
	}


	// open connection to haptic device
	hapticDevice->open();
	if (two_devices)
		hapticDevice2->open();


	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	workspaceScaleFactor = cursorWorkspaceRadius / hapticDeviceInfo.m_workspaceRadius;


	// define a scale factor between tqhe force perceived at the cursor and the
	// forces actually sent to the haptic device
	deviceForceScale = 5.0;

	// create a large sphere that represents the haptic device
	deviceRadius = 0.05; //Nitsan: I changed this from 0.1
	deviceHeight = 0.5; //Nitsan: I added this
	device = new cShapeCylinder(deviceRadius, deviceRadius, deviceHeight);
	//device = new cShapeSphere(deviceRadius); // Nitsan: I changed from sphere to cylinder
	world->addChild(device);
	device->m_material->setWhite();
	device->m_material->setShininess(100);


	//Nitsan: The sphere should mark the middle (for bone strike control)
	/*sphere_radius = 0.05;
	sphere = new cShapeSphere(sphere_radius);

	world->addChild(sphere);
	sphere->m_material->setRed();
	sphere->m_material->setShininess(100);
	*/

	//Nitsan: I  added and commented this (this is the tool from the endoscope)
	/*
	// create a tool (cursor) and insert into the world
	tool = new cToolCursor(world);
	world->addChild(tool);

	// connect the haptic device to the virtual tool
	tool->setHapticDevice(hapticDevice);

	// define the radius of the tool (sphere)
	double toolRadius = 0.01;

	// define a radius for the tool
	tool->setRadius(toolRadius);

	// hide the device sphere. only show proxy.
	tool->setShowContactPoints(false, false);

	// create a white cursor
	tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.0);

	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when
	// the tool is located inside an object for instance.
	tool->setWaitForSmallForce(true);

	// start the haptic tool
	tool->start();


	//--------------------------------------------------------------------------
	// CREATE OBJECTS
	//--------------------------------------------------------------------------

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// properties
	double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

	*/

	// interaction stiffness between tool and deformable model 
	stiffness = 200; //Nitsan: I changed this from 100


	//-----------------------------------------------------------------------
	// COMPOSE THE VIRTUAL SCENE
	//-----------------------------------------------------------------------

	// create a world which supports deformable object
	defWorld = new cGELWorld();
	world->addChild(defWorld);

	// create a deformable mesh
	defObject = new cGELMesh();
	defWorld->m_gelMeshes.push_front(defObject);

	// load model
	bool fileload;
	fileload = defObject->loadFromFile(RESOURCE_PATH("../resources/models/box/box.obj"));
	if (!fileload)
	{
	#if defined(_MSVC)
		fileload = defObject->loadFromFile("../../../bin/resources/models/box/box.obj");
	#endif
	}
	if (!fileload)
	{
		cout << "Error - 3D Model failed to load correctly." << endl;
		close();
		return (-1);
	}

	// set some material color on the object
	cMaterial mat;
	mat.setYellowKhaki(); //Nitsan: I changed this from setWhite();
	mat.setShininess(100); //Nitsan: I changed this from 100
	defObject->setMaterial(mat, true);
	// set object to be transparent
	defObject->setTransparencyLevel(1, true, true); //Nitsan: I changed this from (0.65, true, true)

	//
	//Nitsan: Trying to build another box!!!!
	//

	// create a deformable mesh
	defObject2 = new cGELMesh();
	defWorld->m_gelMeshes.push_back(defObject2); //Nitsan: I changed this from push_front

	// load model
	//bool fileload;
	fileload = defObject2->loadFromFile(RESOURCE_PATH("../resources/models/box/box.obj"));
	if (!fileload)
	{
	#if defined(_MSVC)
		fileload = defObject2->loadFromFile("../../../bin/resources/models/box/box.obj");
	#endif
	}
	if (!fileload)
	{
		cout << "Error - 3D Model failed to load correctly." << endl;
		close();
		return (-1);
	}

	// set some material color on the object
	defObject2->setMaterial(mat, true);
	// set object to be transparent
	defObject2->setTransparencyLevel(1, true, true); //Nitsan: I changed this from (0.65, true, true)


	////// Nitsan: creating ligamentum flavum texture: /////////////
	/*
	double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

	ligamentum = new cMesh();
	cCreatePlane(ligamentum,0.3,0.3);
	ligamentum->createAABBCollisionDetector(0.01);
	world->addChild(ligamentum);
	ligamentum->setLocalPos(0.0, 1.0, 0.0);

	// set graphic properties
	ligamentum->m_texture = cTexture2d::create();
	fileload = ligamentum->m_texture->loadFromFile(RESOURCE_PATH("../../../resources/images/brownboard.jpg"));
	if (!fileload)
	{
		#if defined(_MSVC)
		fileload = ligamentum->m_texture->loadFromFile("../../../../../bin/resources/images/brownboard.jpg");
		#endif
	}
	if (!fileload)
	{
		cout << "Error - Texture image failed to load correctly." << endl;
		close();
		return (-1);
	}

	ligamentum->setUseTexture(true);
	ligamentum->m_material->setWhite();

	// create normal map from texture data
	cNormalMapPtr normalMap2 = cNormalMap::create();
	normalMap2->createMap(ligamentum->m_texture);
	ligamentum->m_normalMap = normalMap2;

	// set haptic properties
	ligamentum->m_material->setStiffness(0.4 * maxStiffness);
	ligamentum->m_material->setStaticFriction(0.2);
	ligamentum->m_material->setDynamicFriction(0.2);
	ligamentum->m_material->setTextureLevel(0.2);
	ligamentum->m_material->setHapticTriangleSides(true, false);
	*/
	/////////////////////////////////////

	//Nitsan: I commented this, this is shadowing

	// let's create a some environment mapping
	/*shared_ptr<cTexture2d> texture(new cTexture2d());
	fileload = texture->loadFromFile(RESOURCE_PATH("../resources/images/shadow.jpg"));
	if (!fileload)
	{
	#if defined(_MSVC)
	fileload = texture->loadFromFile("../../../bin/resources/images/shadow.jpg");
	#endif
	}
	if (!fileload)
	{
	cout << "Error - Texture failed to load correctly." << endl;
	close();
	return (-1);
	}


	// enable environmental texturing
	texture->setEnvironmentMode(GL_DECAL);
	texture->setSphericalMappingEnabled(true);

	// assign and enable texturing
	defObject->setTexture(texture, true);
	defObject->setUseTexture(true, true);

	*/


	// build dynamic vertices
	defObject->buildVertices();
	defObject->setLocalPos(0.0, 0.0, 0.0); //Nitsan: I added this
	//cMatrix3d Rtransform;
	//Rtransform.setAxisAngleRotationDeg(0, 1, 0, 0); // (0,1,0,75)
	//defObject->setLocalRot(Rtransform);
	//Nitsan: Trying to add forces to the 2nd layer
	/*layer1_end = defObject->getBoundaryMin(); //Nitsan: I added this
	//cout << "Y:" << layer1_end.y() << endl; //Nitsan: I added this
	cout << "Min1:" << layer1_end << endl; //Nitsan: I added this
	zlayer1_thickness = layer1_end.z(); //Nitsan: I added this
	cout << "Z1: " << zlayer1_thickness << endl; //Nitsan: I added this*/


	defObject2->setLocalPos(0.0, 0.0, -0.1); //Nitsan: I added this
	defObject2->buildVertices(); //Nitsan: I added this
	//defObject2->setLocalRot(Rtransform);
	/*layer2_end = defObject2->getBoundaryMin(); //Nitsan: I added this
	cout << "Min2:" << layer2_end << endl; //Nitsan: I added this
	zlayer2_thickness = layer2_end.z(); //Nitsan: I added this
	cout << "Z2: " << zlayer2_thickness << endl; //Nitsan: I added this*/

	// Nitsan: set properties for linear spring
	//cGELLinearSpring::s_default_kSpringElongation = 50; // [N/m] //Nitsan: I added this
	//cGELLinearSpring* newSpring = new cGELLinearSpring(); //Nitsan: I added this


	// set default properties for skeleton nodes
	cGELSkeletonNode::s_default_radius = 0.05;  // [m] //Nitsan: I changed this from 0.05
	cGELSkeletonNode::s_default_kDampingPos = 20.0; //Nitsan: I changed this from 2.5
	cGELSkeletonNode::s_default_kDampingRot = 0.8; //Nitsan: I changed this from 0.6
	cGELSkeletonNode::s_default_mass = 0.002; // [kg]
	cGELSkeletonNode::s_default_showFrame = true;
	cGELSkeletonNode::s_default_color.setBlueCornflower();
	cGELSkeletonNode::s_default_useGravity = true;
	cGELSkeletonNode::s_default_gravity.set(0.00, 0.00, -9.81);
	radius = cGELSkeletonNode::s_default_radius;

	// use internal skeleton as deformable model
	defObject->m_useSkeletonModel = true;
	defObject2->m_useSkeletonModel = true; // Nitsan: I added this 

	// create an array of nodes
	for (int y = 0; y < 10; y++) //Nitsan: I changed this from y<10
	{
		for (int x = 0; x < 10; x++) //Nitsan: I changed this from y<10
		{
			cGELSkeletonNode* newNode = new cGELSkeletonNode();
			nodes[x][y] = newNode;
			defObject->m_nodes.push_front(newNode);
			//defObject2->m_nodes.push_front(newNode); //Nitsan: I added this
			newNode->m_pos.set((-0.45 + 0.1*(double)x), (-0.43 + 0.1*(double)y), 0.0); //Nitsan: I changed this from ( (-0.45 + 0.1*(double)x), (-0.43 + 0.1*(double)y), 0.0);
		}
	}

	// set corner nodes as fixed
	nodes[0][0]->m_fixed = true;
	nodes[0][9]->m_fixed = true; //Nitsan: I changed this from [0][9]
	nodes[9][0]->m_fixed = true; //Nitsan: I changed this from [9][0]
	nodes[9][9]->m_fixed = true; //Nitsan: I changed this from [9][9]

	// set default physical properties for links
	cGELSkeletonLink::s_default_kSpringElongation = 25.0;  // [N/m]
	cGELSkeletonLink::s_default_kSpringFlexion = 0.5;   // [Nm/RAD]
	cGELSkeletonLink::s_default_kSpringTorsion = 0.1;   // [Nm/RAD]
	cGELSkeletonLink::s_default_color.setBlueCornflower();

	/* Nitsan: I want to remove the nodes
	// create links between nodes
	for (int y=0; y<9; y++) //Nitsan: I changed this from y<9
	{
	for (int x=0; x<9; x++) //Nitsan: I changed this from x<9
	{
	cGELSkeletonLink* newLinkX0 = new cGELSkeletonLink(nodes[x+0][y+0], nodes[x+1][y+0]);
	cGELSkeletonLink* newLinkX1 = new cGELSkeletonLink(nodes[x+0][y+1], nodes[x+1][y+1]);
	cGELSkeletonLink* newLinkY0 = new cGELSkeletonLink(nodes[x+0][y+0], nodes[x+0][y+1]);
	cGELSkeletonLink* newLinkY1 = new cGELSkeletonLink(nodes[x+1][y+0], nodes[x+1][y+1]);
	defObject->m_links.push_front(newLinkX0);
	defObject->m_links.push_front(newLinkX1);
	defObject->m_links.push_front(newLinkY0);
	defObject->m_links.push_front(newLinkY1);
	}
	}

	// connect skin (mesh) to skeleton (GEM)
	defObject->connectVerticesToSkeleton(false);
	//defObject2->connectVerticesToSkeleton(false); //Nitsan: I added this

	// show/hide underlying dynamic skeleton model
	defObject->m_showSkeletonModel = false;
	//defObject2->m_showSkeletonModel = false; //Nitsan: I added this
	*/


	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	cFontPtr font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	camera->m_frontLayer->addChild(labelRates);
	labelRates->m_fontColor.setBlack();

	//Nitsan: marking the (0,0) point
	//marker = new cLabel(font); //Nitsan: I added this to mark the (0,0) point
	//camera->m_frontLayer->addChild(marker);
	//marker->m_fontColor.setRed();

	// create a background
	cBackground* background = new cBackground();
	camera->m_backLayer->addChild(background);

	// set background properties
	background->setCornerColors(cColorf(1.00f, 1.00f, 1.00f),
		cColorf(0.95f, 0.95f, 0.95f),
		cColorf(0.85f, 0.85f, 0.85f),
		cColorf(0.80f, 0.80f, 0.80f));


	// Nitsan: I added this next part
	// create a scope to plot haptic device force data
	scope = new cScope();
	camera->m_frontLayer->addChild(scope);
	scope->setLocalPos(0.0, 400); //Nitsan: I changed this from (100,60)
	scope->setRange(-10, 10); //Nitsan: I changed this from (-0.1,0.1)
	scope->setSignalEnabled(true, true, true, false);
	scope->setTransparencyLevel(0.7);
	//-----------------------------------------------------------------------
	// START SIMULATION
	//-----------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	//int i;
	//hapticsLoop:
	//for (i = 0; i < 1; i++) { //Nitsan: i'm not sure what this loop does now....
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	//cout << i << endl;
	//}

	/*if (injection) {
		hapticsThread->stop();
		injection = false;

		goto hapticsLoop;
		}*/
	// setup callback when application exits
	atexit(close);


	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);

		// render graphics
		updateGraphics();

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		freqCounterGraphics.signal(1);
	}

	// close window
	glfwDestroyWindow(window);

	// terminate GLFW library
	glfwTerminate();

	// exit
	return 0;
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;

	// update position of scope
	scope->setSize(width*0.3, 180); //Nitsan: I added this

}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------
//KEYBOARD
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		//hapticsThread->stop();
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
		string s("999");

		if (s.compare(Subject_Name)) {
			fclose(Data_file); //Nitsan: I added this
			fclose(Data_file2);
			fclose(Success_file);
		}
	}

	// option - show/hide skeleton
	/* NitsanL I want to remove the nodes
	else if (a_key == GLFW_KEY_S)
	{
	defObject->m_showSkeletonModel = !defObject->m_showSkeletonModel;
	}
	*/
	// option - toggle fullscreen
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

	// option - toggle vertical mirroring
	else if (a_key == GLFW_KEY_M)
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}

	else if (a_key == GLFW_KEY_L) //Nitsan: creating the channel (L is for leaving the needle)
	{
		hapticDevice->getPosition(pos_L);
		create_channel = true;
	}

	else if (a_key == GLFW_KEY_P) //Nitsan: un-creating the channel (Z is for picking up the needle)
	{
		create_channel = false;
	}

	// Nitsan: option - state that you have reached ES (and later it will also be to start the next trial)
	else if (a_key == GLFW_KEY_D)
	{
		StopTrial();

	}
	else if (a_key == GLFW_KEY_S) { //Nitsan: (S=Second) user has asked to start using the LOR syringe
		force_in_2 = true;
	}

	else if (a_key == GLFW_KEY_N) { //Nitsan: (N=No) user is NOT holding the 2nd device, dont give forces yet
		startForce2 = false;
		cout << endl;
		cout << "Forces in 2nd device deactivated" << endl;
	}

	else if (a_key == GLFW_KEY_Y) { //Nitsan: (Y=Yes) user confirmed holding 2nd device, so we can start giving the forces
		startForce2 = true;
		cout << endl;
		cout << "Forces in 2nd device activated" << endl;
	}
}
//---------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	hapticDevice->close();
	if (two_devices)
		hapticDevice2->close();

	// delete resources
	delete hapticsThread;
	delete world;
	delete handler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
	labelRates->setFontScale(1); //Nitsan: I added this


	//marker->setText("*"); //Nitsan: I added this
	//marker->setLocalPos((int)(0.5 * (width - marker->getWidth())), 500); //Nitsan: I added this
	//marker->setFontScale(2);

	/////////////////////////////////////////////////////////////////////
	// UPDATE DEFORMABLE MODELS
	/////////////////////////////////////////////////////////////////////

	// update skins deformable objects
	defWorld->updateSkins(true);


	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// wait until all GL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{

	getThickness();
	//Nitsan: get the time for the fprintf
	Timer timer;
	timer.reset();

	// initialize precision clock
	cPrecisionClock clock;
	clock.reset();

	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	//Nitsan: calculating the dependancy in the angle. We need to remember 2 things: the d that we have went so far (accumulating), and the previous point.


	//FILTER:
	double N = 15; //Nitsan: order of the filter
	cVector3d vel[15];
	bool vel_bool[15];
	int i;

	for (i = 0; i < N; i++) {
		vel_bool[i] = false;
		vel[i].zero();
	}


	cVector3d previous_vel; //Nitsan vel[n-1]
	hapticDevice->getLinearVelocity(previous_vel);
	vel[1] = previous_vel;

	//end FILTER

	cVector3d previous_pos;
	hapticDevice->getPosition(previous_pos);
	previous_pos.mul(workspaceScaleFactor);
	cVector3d previous_dir; //Nitsan: we need this to be initilized out of the loop!
	previous_dir.zero();
	cVector3d deltaP;
	deltaP.zero();


	//previous_vel.mul(workspaceScaleFactor);
	//double d = previous_pos.x(); //Nitsan: I think we dont need this at all

	bool entered_back = false; //Nitsaqn; this needs to be initialized in the first entrance into the back
	cVector3d start_pos;
	start_pos.zero();

	//Nitsan: forces for the LOR syringe
	cVector3d force2;
	force2.zero();

	// Nitsan: the configuration!!! (old or new)
	bool direction = false; //Nitsan: false=y, true=x

	//Nitsan: get to desired position
	bool check;
	if (firstTime) {
		firstTime = false;
		check = StartingPosition(20, 20, direction); //Nitsan: if this is the 1st trial, then we need only 10,10 
	}
	else
		check = StartingPosition(20, 20, direction); //Nitsan: otherwise, we need more


	if (!check)
		cout << "Error in Start Positioning" << endl;
	//cout << check << endl;

	//Nitsan: BOOL DIRECTORY
	bool create_layers = true;
	bool create_bones = false; //Nitsan: keep it this way for now - we will try to print the bones instead
	bool route_memory = false; //Nitsan: this is the bool for the calculations that remember where we came from etc.
	bool create_steady_channel = false; //Nitsan: trying to create the force channel all along and not only when letting go
	bool print_vel = false;
	bool filter_vel = false || route_memory; //Nitsan: or manual desicion, or if we want route memory then for sure filter velocity
	//bool force_in_2 = true;

	
	//int i;
	for (i = 0; i < 4; i++) {
	firstTimePert[i] = true;
	}

	double counter = 0;
	//Nitsan: minus forces in LOR or zero forces in ES and after
	bool ZERNEG = false; //false = zero
	// true = negative (the needle falls)

	double layername = 0; // 0 is out of back
	double layerthickness = 0; //just something initial


	// main haptic simulation loop
	while (simulationRunning)
	{

		// stop clock
		double time = cMin(0.001, clock.stop());

		// restart clock
		clock.start(true);

		// read position from haptic device
		cVector3d pos;
		hapticDevice->getPosition(pos);
		pos.mul(workspaceScaleFactor);

		//Nitsan: robot 2
		cVector3d pos2;
		hapticDevice2->getPosition(pos2);
		pos2.mul(workspaceScaleFactor);

		cVector3d velocity2;
		hapticDevice2->getLinearVelocity(velocity2); //Nitsan: measuring velocity

		hapticDevice->getLinearVelocity(velocity); //Nitsan: measuring velocity
		vel[0] = velocity;
		//velocity.mul(workspaceScaleFactor); //Nitsan: Im not sure that we will need this


		//cMatrix3d Rtransform;
		//Rtransform.setAxisAngleRotationDeg(0, 1, 0, 75);
		//Rtransform.mul(pos);
		device->setLocalPos(pos);
		//sphere->setLocalPos(0.0, 3.0, 1.0);

		///////////////////////////////////////////////
		cMatrix3d rot;
		//hapticDevice->getRotation(rot);
		cMatrix3d rot2;
		cMatrix3d Rtool;
		Rtool.setAxisAngleRotationDeg(0, 1, 0, 35);
		double rotrot[3][3] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
		rot.set(rotrot);
		rot.mulr(Rtool, rot2);
		device->setLocalRot(rot2);
		//////////////////////////////////////////////

		//cout << "Pos: " << pos << endl; //Nitsan: I added this

		// clear all external forces
		defWorld->clearExternalForces();

		// compute reaction forces
		cVector3d force(0.0, 0.0, 0.0);
		force2.zero();
		/* Nitsan: I want to remove the nodes
		for (int y=0; y<10; y++) //Nitsan: I changed this from y<10
		{
		for (int x=0; x<10; x++) //Nitsan: I changed this from x<10
		{
		cVector3d nodePos = nodes[x][y]->m_pos;
		cVector3d f = computeForce(pos, deviceRadius, nodePos, radius, stiffness);
		cVector3d tmpfrc = -1.0 * f;
		nodes[x][y]->setExternalForce(tmpfrc);
		force.add(f);
		}
		}

		*/
		//Nitsan: Trying to add forces to the 2nd layer
		//Nitsan: I dont think that we need this anymore but let's leave it for now
		//layer1_end = defObject->getBoundaryMin(); //Nitsan: I added this
		//layer2_end = defObject2->getBoundaryMin(); //Nitsan: I added this
		//cout << "Y:" << layer1_end.y() << endl; //Nitsan: I added this
		//cout << "Min:" << layer1_end << endl; //Nitsan: I added this
		//zlayer1_thickness = layer1_end.z(); //Nitsan: I added this
		//zlayer2_thickness = layer2_end.z(); //Nitsan: I added this
		//cout << "Z: " << zlayer1_thickness << endl; //Nitsan: I added this

		// Nitsan: checking 
		//cout << "Entrance Point: " << enter_point << endl;


		freqCounterHaptics.setTimePeriod(1 / 500); //Nitsan: trying to set the feq to be 500 Hz

		double temp_force;
		double angle = 3.141592654 / 12; //Nitsan: 15 degrees
		double start_point = 15; //Nitsan: 35 if we use lengthener

		posZ = (pos.z()) / 0.01; //Nitsan: if pos.z() is in dm -> we need pos.z()/10^-2;
		posX = (pos.x()) / 0.01; //Nitsan: dm-> mm

		//scaling = (1.5/3.6767); //Nitsan: this can also work
		scaling = (1.8 / 3.6767)*0.65; //originally without 0.75 //Nitsan: fixing the size of the forces not to get saturation of omni 				
		//Nitsan: according to Brazil et al. the 3.6767 is to scale because of robot limits

		angle = 0; // Nitsan: if this line is commented then the angle is 15 deg
		double posX_transformed;
		posX_transformed = posX*cos(angle) + posZ*sin(angle); //Nitsan: rotation of -15 deg around y axis
		double posZ_transformed;
		posZ_transformed = -posX*sin(angle) + posZ*cos(angle); //Nitsan
		//posZ_transformed = posZ; //Nitsan
		double posY = pos.y() / 0.01;
		double temp_forceY;

		

		//double signY;
		//double signZ;
		//signY = posY / abs(posY);
		//signZ = posZ_transformed / abs(posZ_transformed);
		// Nitsan: we use the sign because to fix a neg num to zero we need to substract and a pos to add



		//////////////////////DIRECTIONING!!!!
		cVector3d dir;
		double d;
		if (direction) { 		//Nitsan: changing configuration: X direction

			dir.set(1, 0, 0);
			d = posX_transformed;
		}
		else {	//Nitsan: changing configuration: Y direction

			dir.set(0, 1, 0);
			d = posY;
		}
	
		//cout << pos.y() << endl;
		cVector3d force_1; //Nitsan: this is the same as force, but without the channel (relevant for force2)
		force_1.zero();
		cVector3d actual_force;
		cVector3d actual_force2;
		//cout << posZ_transformed << endl;
		force.zero();
		//cout << "z: " << posZ_transformed << " y: " << posY << endl;
		
		double bone = 10;
		//double theta;
		//theta = atan2(posZ_transformed, posY);
		
		//Nitsan: if we go with forces that are always perpendicular to the patient's back:
		//dir.set(cos(angle),0,sin(angle));
		// Nitsan: if we go ahead with an angle that affects the location but not the direction of the force, then:


		double scalingCO = scaling_coef[(int)TrialNum - 1];

		cVector3d filtered_vel;
		if (filter_vel){
			filtered_vel.zero();

			for (i = 0; i < N; i++) {
				filtered_vel.add(vel[i] / N);
			}
		}



		//Nitsan: calculate the difference between this location and the previous one
		if (route_memory) { // States whether we are using the complicated calculations about the angle and memory or not
			/*deltaP = pos - previous_pos;
			//double deltaX = (deltaP.x()*cos(angle)+deltaP.z()*sin(angle))/0.01;
			//double deltaY = deltaP.y()/0.01;
			//double deltaZ = (deltaP.z()*cos(angle)-deltaP.x()*sin(angle))/0.01;
			//d = d + deltaX; //update d (the penetration on the transformed x axis) at each iteration.
			//posX_transformed = d; //if we are using these calcs, then instead of calculating with posX_transformed, we need to use this d that's updating all the time, for the polinomial
			//deltaP.set(deltaX, 0, deltaZ);
			deltaP.set(deltaP.x(),0,deltaP.z());
			deltaP.normalize(); //Nitsan: this is like deviding by the size of the vector -> gives us the direction of the vector
			if (deltaP.x()==0 && deltaP.z()==0) {
			//dir.set(previous_dir.x(), 0, previous_dir.z()); //Nitsan: if this dir isnt good, lets use the last one that worked
			dir = previous_dir;
			}
			else {
			dir.set(-deltaP.x(), 0, -deltaP.z()); //Nitsan: why does this cause problems?
			//previous_dir.set(-deltaP.x(), 0, -deltaP.z()); //Nitsan: in case this is good, we want to keep it
			previous_dir = dir;
			}*/

			cVector3d velocity_dir;
			//Nitsan: filtered velocity
			velocity_dir = velocity;
			if (filter_vel)
				velocity_dir = filtered_vel;
			velocity_dir.set(velocity_dir.x(), 0, velocity_dir.z());
			velocity_dir.normalize();
			dir = -velocity_dir;

		}

		//cVector3d temp_force_2(0, 1.8, 0);
		//cout << pos.y() << endl;
		//if (pos.y() < 0.5 && pos.y() > -0.5) { // Nitsan: this will define that we are in the box in terms of x,y
		if (create_bones) {
			if (d < start_point - 20 && d > start_point - 41.18 && ((posZ_transformed > bone || posZ_transformed < -bone) || (posY > bone || posY < -bone))){ //Nitsan: this means that we hit the bone (x: only between interspinous ligament and ligamentum flavum)

				// Nitsan: creating the bones (walls, 100 N/m stiffness)
				temp_force = (-posZ_transformed)*0.001 * 100;
				temp_forceY = (-posY)*0.001 * 100;
				//temp_forceY = 0;

				///*
				if (posZ_transformed > bone || posZ_transformed < -bone) {
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle));
					force.add(temp_force*dir.x(), 0.0, temp_force*dir.z());
				}
				if (posY > bone || posY < -bone) {
					//force.add(temp_forceY*cos(angle), 0.0, temp_forceY*sin(angle));
					force.add(temp_force*dir.x(), 0.0, temp_force*dir.z());

					//*/
					//force.set(0.0, temp_forceY*1.5, temp_force*1.5);
					// Bone strike showing in text:
					labelRates->setText("Bone Strike!");
					labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 500);
					labelRates->setFontScale(3);
				}
			}
			else
				force.zero();
		}
			
			if (create_layers) {

				//Nitsan: keep the point of entrance into the back

				if (create_steady_channel && abs(start_point - (d)) < 0.25 && !entered_back) { //Nitsan: we want to record the position of entering the back so that we make the channel accordingly
					start_pos = pos;
					entered_back = true;
				}
				//cout << start_pos.x() << ", " << start_pos.y() << ", " << start_pos.z() << ", " << endl;

				if (d < start_point && d > start_point - 56.98 && (create_channel || create_steady_channel)){ // Nitsan: check if we are in the epidural region at all, and then take a look if we need a channel

					x_channel = pos_L.x() / 0.01;
					y_channel = pos_L.y() / 0.01;
					z_channel = pos_L.z() / 0.01;
					k_channel = 0.001 * 200; //Nitsan: with unit fixing this is 200 N/m

					double velX = velocity.x() / 0.01;
					double velY = velocity.y() / 0.01;
					double velZ = velocity.z() / 0.01;
					b_channel = 0.001 * 200;
					//Nitsan: measuring velocity!


					//cout << theta << endl;


					if (create_steady_channel && !injection) { //Nitsan: only if we want channel, and injection==0 which means that we are not in the middle of getting the needle to initail position (between trials)
						//Nitsan: we want inES==-1 so that after LOR (in ES and after), we will not have the channel anymore
						k_channel = 0.001 * 150;
						b_channel = 0.001 * 150;
						// option 1: pos-prevpos, vel-prevvel 
						//force.set(0.0, -k_channel*(posY - previous_pos.y() / 0.01) - b_channel*(velY-previous_vel.y()/0.01), -k_channel*(posZ_transformed - previous_pos.z() / 0.01) - b_channel*(velZ-previous_vel.z()/0.01));
						// option 2: pos-prevpos, vel
						//force.set(0.0, -k_channel*(posY - previous_pos.y() / 0.01) - b_channel*velY, -k_channel*(posZ_transformed - previous_pos.z() / 0.01) - b_channel*velZ);
						// option 3: pos-start_pos, vel-prevvel
						if (direction)
							force.set(0.0, -k_channel*(posY - start_pos.y() / 0.01) - b_channel*(velY - previous_vel.y() / 0.01), -k_channel*(posZ_transformed - start_pos.z() / 0.01) - b_channel*(velZ - previous_vel.z() / 0.01));
						else
							force.set(-k_channel*(posX_transformed - start_pos.x() / 0.01) - b_channel*(velX - previous_vel.x() / 0.01),0.0, -k_channel*(posZ_transformed - start_pos.z() / 0.01) - b_channel*(velZ - previous_vel.z() / 0.01));


					}

					if (create_channel) {
						k_channel = 0.001 * 200;
						b_channel = 0.001 * 200;
						force.set(0.0, -k_channel*(posY - y_channel) - b_channel*velY, -k_channel*(posZ_transformed - z_channel) - b_channel*velZ);
					}
					//force.set(0.0, (-k_channel*(posY - y_channel) - b_channel*velY)*cos(theta), (-k_channel*(posZ_transformed - z_channel) - b_channel*velZ)*sin(theta));
					//cout << "x0: " << x_channel << " y0: " << y_channel << " z0: " << z_channel << endl;
				}
				if (create_channel == 0 && create_steady_channel == 0) //Nitsan: if we are not interested in a channel at all
					force.zero();
				if (d < start_point + 0 && d > start_point - thick[0]) { // Nitsan: this defines the thickness of the layer (SKIN)
					//force.zero();
					posX_fixed = d - start_point;
					temp_force = (0.0235 + 0.0116*(-posX_fixed) / scalingCO - 0.0046*(pow(-posX_fixed, 2)) / pow(scalingCO, 2) + 0.0025*(pow(-posX_fixed, 3)) / pow(scalingCO,3))*scaling;
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle)); //Nitsan: according to Brazil et al. the 3.6767 is to scale because of robot limits
					force.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					force_1.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					inES = -1; //Nitsan: -1 states that we are before the ES
					dural_puncture = false; //Nitsan: sometimes the needle starts from beneath and it counts as dural puntcure, so let's start the count from here
					//force2 = temp_force_2;
					
					layername = 1; //skin
					layerthickness = thick[0];
				}
				if (d <= start_point - thick[0] && d > start_point - thick[1]) { // Nitsan: (FAT)
					//force.zero();
					posX_fixed = d + thick[0] - start_point;
					temp_force = (6.0372 + 0.4516*(-posX_fixed) / scalingCO - 0.5287*(pow(-posX_fixed, 2)) / pow(scalingCO,2))*scaling;
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle));
					force.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					force_1.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					inES = -1; //Nitsan: -1 states that we are before the ES
					//force2 = temp_force_2;

					layername = 2; // fat
					layerthickness = thick[1]-thick[0];

					// Nitsan: add pertubation in trial 77
					if (TrialNum == 77) {
						for (counter = 0; counter < 200; counter++) {
							createPertubation(firstTimePert[0]);
						}
						firstTimePert[0] = false;
					}
				}
				if (d <= start_point - thick[1] && d > start_point - thick[2]) { // Nitsan: (MS before)
					//force.zero();
					posX_fixed = d + thick[1] - start_point;
					temp_force = (1.9736 + 0.8287*(-posX_fixed) / scalingCO + 0.1078*(pow(-posX_fixed, 2)) / pow(scalingCO,2))*scaling;
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle));
					force.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					force_1.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					inES = -1; //Nitsan: -1 states that we are before the ES
					//force2 = temp_force_2;

					layername = 3; //MS
					layerthickness = thick[3] - thick[1];
				}
				if (d <= start_point - thick[2] && d > start_point - thick[3]) { // Nitsan: (MS after)
					//force.zero();
					posX_fixed = d + thick[2] - start_point;
					temp_force = (4.354 - 2.2543*(-posX_fixed) / scalingCO + 0.2902*(pow(-posX_fixed, 2)) / pow(scalingCO,2))*scaling;
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle));
					force.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					force_1.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					inES = -1; //Nitsan: -1 states that we are before the ES
					//force2 = temp_force_2;

					// Nitsan: add pertubation in trial 78
					if (TrialNum == 78) {
						for (counter = 0; counter < 200; counter++) {
							createPertubation(firstTimePert[1]);
						}						
						firstTimePert[1] = false;
					}

				}
				if (d <= start_point - thick[3] && d > start_point - thick[4]) { // Nitsan: (IL before)
					//force.zero();
					posX_fixed = d + thick[3] - start_point;
					temp_force = (4.4062 + 0.9598*(-posX_fixed) / scalingCO)*scaling;
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle));
					force.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					force_1.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					inES = -1; //Nitsan: -1 states that we are before the ES
					//force2 = temp_force_2;
					
					layername = 4; //IL
					layerthickness = thick[5] - thick[3];

				}
				if (d <= start_point - thick[4] && d > start_point - thick[5]) { // Nitsan: (IL after)
					//force.zero();
					posX_fixed = d + thick[4] - start_point;
					temp_force = (7.467)*scaling;
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle));
					force.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					force_1.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					inES = -1; //Nitsan: -1 states that we are before the ES
					//force2 = temp_force_2;

					// Nitsan: add pertubation in trial 80
					if (TrialNum == 80) {
						for (counter = 0; counter < 200; counter++) {
							createPertubation(firstTimePert[2]);
						}						
						firstTimePert[2] = false;
					}

				}
				if (d <= start_point - thick[5] && d > start_point - thick[6]) { // Nitsan: (LF before)
					//force.zero(); q
					posX_fixed = d + thick[5] - start_point;
					temp_force = (7.467 + 1.5029*(-posX_fixed) / scalingCO - 0.0583*(pow(-posX_fixed, 2)) / pow(scalingCO,2))*scaling;
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle));
					// Nitsan: adjust ligamentum flavum fibers: 0.25 factor
					force.add(temp_force*dir.x(), (temp_force+0*sin(10000*posX_fixed))*dir.y(), temp_force*dir.z());
					//force.add(temp_force*dir.x(), (temp_force + 0.0001*createBuldges(posZ_transformed).y())*dir.y(), temp_force*dir.z());
					force_1.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					inES = -1; //Nitsan: -1 states that we are before the ES
					//force2 = temp_force_2;

					layername = 5; //LF
					layerthickness = thick[7] - thick[5];
				}
				if (d <= start_point - thick[6] && d > start_point - thick[7]) { // Nitsan: (LF after)
					//force.zero();
					posX_fixed = d + thick[6] - start_point;
					temp_force = (12.133 - 0.1693*(-posX_fixed) / scalingCO - 0.1177*(pow(-posX_fixed, 2)) / pow(scalingCO,2))*scaling;
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle));
					force.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					force_1.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					inES = 0; //Nitsan: -1 states that we are before the ES
					//force2 = temp_force_2;

					// Nitsan: add pertubation in trial 79
					if (TrialNum == 79) {
						for (counter = 0; counter < 200; counter++) {
							createPertubation(firstTimePert[3]);
						}
						firstTimePert[3] = false;
					}
				}
				if (d <= start_point - thick[7]) { // Nitsan: (ES)
					//force.zero();
					posX_fixed = d + thick[7] - start_point;
					temp_force = (0.0)*scaling;
					force.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());
					force_1.add(temp_force*dir.x()/1.15, temp_force*dir.y()/1.15, temp_force*dir.z()/1.15);
					//force.add(temp_force*cos(angle), 0.0, temp_force*sin(angle));
					/*if (ZERNEG) //true= negative forces
						force_1.add(-temp_force*dir.x(), -temp_force*dir.y(), -temp_force*dir.z());
					else //false= no forces
						force_1.zero();*/
					inES = 0; //Nitsan: 0 states that we are inside the ES
					//force2.zero();

					if (!create_steady_channel) //Nitsan: if we cancel this then comment only the if
						force.add(temp_force*dir.x(), temp_force*dir.y(), temp_force*dir.z());

					layername = 6; //ES
					layerthickness = thick[8] - thick[7];

				}
				if (d <= start_point - thick[8]) { //Nitsan: in case of dural tap: according to Brazil et al., ES is 8.6 mm
					posX_fixed = d + thick[8] - start_point;
					labelRates->setText("Dural Puncture!");
					labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 500);
					labelRates->setFontScale(3);
					inES = 1; //Nitsan: 1 states that we are after the ES (dural puncture);
					dural_puncture = true; //Nitsan: dura has been punctured!
					//force_1.zero();
					
					layername = 7; //Dura Materq
				}
				/*if (d <= start_point - (thick[8]+28)) { //Nitsan: twice the std of the error in the pilot from april 2021
					//cout << "Passed 2 std" << endl;
					StopTrial();
				}*/
			}
		//}
		// integrate dynamics
		defWorld->updateDynamics(time);

		// scale force
		force.mul(deviceForceScale / workspaceScaleFactor);

		// send forces to haptic device
		hapticDevice->setForce(force);


		//force2.set(force.y(), 2 * force.x(), 0.5);
		double scaling2 = 1.15; //Nitsan: originally 1.5 or 4
		force2.set(scaling2*force_1.x(), scaling2*force_1.y(), scaling2*force_1.z()); //Nitsan: using force_1 prevents problems when using a steady force channel

		//Nitsan: set forces in 2nd device
		if (force_in_2) { //Nitsan: user asks to start giving forces in 2nd device
			if (print_force2) { //Nitsan: 1st time, we ask him to hold the device
				cout << endl;
				cout << "Force in 2nd device activated, confirm holding 2nd device: " << endl;
				cout << "[N] no" << endl;
				cout << "[Y] yes" << endl;
				//cin >> startForce2;
				print_force2 = false; //Nitsan: we want to print this only once
			}
			if (startForce2){ //Nitsan: only after he is holding
				hapticDevice2->setForce(force2);
				if (print_exit_force2){
					cout << endl;
					cout << "If you wish to stop enabling forces in 2nd device, hit [N]" << endl;
					print_exit_force2 = false; //Nitsan: we want to print this only once
				}
			}
			else {
				force2.zero();
				hapticDevice2->setForce(force2);
			}
		}
		// signal frequency counter
		freqCounterHaptics.signal(1);

		//Nitsan: I added this
		// update information to scope
		//Data_file = fopen(filename, "wt"); //Nitsan: I added this

		hapticDevice->getForce(actual_force);
		hapticDevice2->getForce(actual_force2);
		//cout << actual_force.x() << endl; //Nitsan: check the bones
		scope->setSignalValues(actual_force.x()); //Nitsan: I want only to show the z force

		//double size;
		//size = actual_force.length();
		//cout << size << endl;

		//Nitsan: checking the filter
		//cVector3d filtered_vel = (velocity + previous_vel + vel_2 + vel_3 + vel_4) / (N + 1);


		string s("999");

	if (s.compare(Subject_Name)) {
			//fprintf(Data_file, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", velocity.x(), velocity.y(), velocity.z(), previous_vel.x(), previous_vel.y(), previous_vel.z(), vel_2.x(), vel_2.y(), vel_2.z(), vel_3.x(), vel_3.y(), vel_3.z(), vel_4.x(), vel_4.y(), vel_4.z(), filtered_vel.x(), filtered_vel.y(), filtered_vel.z());
			if (print_vel) {
				for (i = 0; i < N; i++) {
					fprintf(Data_file, "%f ", vel[i].x());
				}
				fprintf(Data_file, "%f \n", filtered_vel.x());
			}
			else {

				fprintf(Data_file, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n", timer.get(), pos.x(), pos.y(), pos.z(), velocity.x(), velocity.y(), velocity.z(), actual_force.x(), actual_force.y(), actual_force.z(), freqCounterHaptics.getFrequency(), layername, layerthickness); //Nitsan: I added this, add later velocities!
				//Nitsan: robot2
				fprintf(Data_file2, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n", timer.get(), pos2.x(), pos2.y(), pos2.z(), velocity2.x(), velocity2.y(), velocity2.z(), actual_force2.x(), actual_force2.y(), actual_force2.z(), freqCounterHaptics.getFrequency(), layername, layerthickness); //Nitsan: I added this, add later velocities!

			}

		}

		//cout << pos.x() << " " << pos.y() << " " << pos.z() << endl;
		//counter++; //Nitsan: I added this
		previous_pos = pos;

		//Nitsan: for FILTER
		if (filter_vel) {
			for (i = N - 1; i > 0; i--) {
				if (!vel_bool[i] && (vel[i].x() == 0 && vel[i].y() == 0 && vel[i].z() == 0) && (vel[i - 1].x() != 0 || vel[i - 1].y() != 0 || vel[i - 1].z() != 0))
					vel_bool[i] = true;
				if (vel_bool[i])
					vel[i] = vel[i - 1];
			}
		}

		//Nitsan: skip to the next trial
		if (injection) { //the injection states that the trainee has injected the medicine, thus finishing this trial
			injection = false;
			if (TrialNum < NumTrials) {
				TrialNum++;
				cout << endl << "Trial Num: " << TrialNum << endl;

				string s("999");

				if (s.compare(Subject_Name)) {
					fclose(Data_file); //Nitsan: I added this
					fclose(Data_file2);
				}
				OpenDataFile(TrialNum);

				updateHaptics();
			}
			else {
				StartingPosition(20, 20, direction); //Nitsan: we need higher k,b than from the beggining if we passed the ligamentum flavum, to get to original position
				cout << "Simulation over, hit [Q] to exit" << endl;
			}

			//glfwSetWindowShouldClose(a_window, GLFW_TRUE);

		}

		//fclose(Data_file); //Nitsan: I added this
	}

	// exit haptics thread
	simulationFinished = true;
}

//---------------------------------------------------------------------------

cVector3d computeForce(const cVector3d& a_cursor,
	double a_cursorRadius,
	const cVector3d& a_spherePos,
	double a_radius,
	double a_stiffness)
{
	// compute the reaction forces between the tool and the ith sphere.
	cVector3d force;
	force.zero();



	cVector3d vSphereCursor = a_cursor - a_spherePos;

	// check if both objects are intersecting
	if (vSphereCursor.length() < 0.0000001)
	{
		return (force);
	}

	if (vSphereCursor.length() > (a_cursorRadius + a_radius))
	{
		return (force);
	}

	// compute penetration distance between tool and surface of sphere
	double penetrationDistance = (a_cursorRadius + a_radius) - vSphereCursor.length();
	cVector3d forceDirection = cNormalize(vSphereCursor);
	force = cMul(penetrationDistance * a_stiffness, forceDirection);

	// return result
	return (force);
}

//---------------------------------------------------------------------------
//Nitsan: I added this function to write to file
void OpenDataFile(double TrialNumber) {

	// Nitsan: This function basically just opens the file that we need and gives the "title" line.


	//Nitsan: time
	SYSTEMTIME st;
	GetLocalTime(&st);
	double counter = 0;
	double counter2 = 0;

	string s("999");

	if (!s.compare(Subject_Name)) {
		return;
	}

	char foldername[200];
	sprintf(foldername, ".\\DataLogs\\%s_%02d.%02d.%02d", Subject_Name, st.wDay, st.wMonth, st.wYear);
	//_mkdir(foldername);
	// Nitsan: print the name of the file into the spring
	//sprintf(filename, "./DataLogs/Subject_%s_%02d.%02d.%d_%02d.%02d_Touhy.txt", Subject_Name, st.wDay, st.wMonth, st.wYear, st.wHour, st.wMinute);
	//sprintf(filename2, "./DataLogs/Subject_%s_%02d.%02d.%d_%02d.%02d_LOR.txt", Subject_Name, st.wDay, st.wMonth, st.wYear, st.wHour, st.wMinute);

	sprintf(filename, "./%s/%s_%d_Touhy.txt", foldername, Subject_Name, (int)TrialNumber);
	sprintf(filename2, "./%s/%s_%d_LOR.txt", foldername, Subject_Name, (int)TrialNumber);
	//cout << "folder trial num: " << TrialNum << TrialNumber << endl;
	//filename = "./DataLogs/Subject_" + Subject_Name;

	//Nitsan: prevent deleting an existing file (for a certain trial num) if it already exists
	while (FILE* file = fopen(filename, "r")) {
		fclose(file);
		counter++;
		sprintf(filename, "./%s/%s_%d_Touhy_repeat_%d.txt", foldername, Subject_Name, (int)TrialNumber, (int)counter);
	}

	while (FILE* file2 = fopen(filename2, "r")) {
		fclose(file2);
		counter2++;
		sprintf(filename2, "./%s/%s_%d_LOR_repeat_%d.txt", foldername, Subject_Name, (int)TrialNumber, (int)counter2);
	}

	//cout << filename << endl;
	Data_file = fopen(filename, "wt");
	Data_file2 = fopen(filename2, "wt");
	//cout << Data_file << endl; //Nitsan: this is just to check if we were able to create the file at all
	if (Data_file) {
		fprintf(Data_file, "Time Px Py Pz Vx Vy Vz Fx Fy Fz freq layer_num layer_thickness\n"); //Nitsan: Do we really want all this?

	}
	else
		cout << Data_file << endl;

	if (Data_file2) {
		fprintf(Data_file2, "Time Px Py Pz Vx Vy Vz Fx Fy Fz layer_num layer_thickness\n"); //Nitsan: Do we really want all this?

	}
	else
		cout << Data_file2 << endl;
	//fclose(Data_file); //Nitsan: comment this when we actually wrtie something... this is just for validation



}

//----------------------------------------------------------------------
void OpenSuccessFile(double TrialNumber) {
	//Nitsan: this is the file that denotes the success or epidural failure or dural puncture

	//Nitsan: time
	SYSTEMTIME st;
	GetLocalTime(&st);

	int counter = 0;
	string s("999");

	if (!s.compare(Subject_Name)) {
		return;
	}

	char foldername[200];
	sprintf(foldername, ".\\DataLogs\\%s_%02d.%02d.%02d", Subject_Name, st.wDay, st.wMonth, st.wYear);
	//_mkdir(foldername);

	sprintf(success_filename, "./%s/Success_%s_%02d.%02d.%d_FromTrial_%d.txt", foldername, Subject_Name, st.wDay, st.wMonth, st.wYear, (int)TrialNumber);



	while (FILE* file3 = fopen(success_filename, "r")) {
		fclose(file3);
		counter++;
		sprintf(success_filename, "./%s/Success_%s_%02d.%02d.%d_FromTrial_%d_repeat_%d.txt", foldername, Subject_Name, st.wDay, st.wMonth, st.wYear, (int)TrialNumber, (int)counter);
	}


	Success_file = fopen(success_filename, "wt");
	if (Success_file) {
		fprintf(Success_file, "Trial Number    Dural_Puncture    Failed_Epidural    Success\n"); //Nitsan: Do we really want all this?
	}
	else
		cout << Success_file << endl;
}

//-----------------------------------------------------------------------
void OpenFolder(){

	SYSTEMTIME st;
	GetLocalTime(&st);

	string s("999");

	if (!s.compare(Subject_Name)) {
		return;
	}

	char foldername[200];
	sprintf(foldername, ".\\DataLogs\\%s_%02d.%02d.%02d", Subject_Name, st.wDay, st.wMonth, st.wYear);
	_mkdir(foldername);

}

//-------------------------------------------------------------------------

bool StartingPosition(double k, double b, bool direction){

	bool gotToPosition = false;
	//Nitsan: bool direction- true for x, false for y

	cVector3d Desired_Pos;
	if (direction)
		Desired_Pos.set(0.7, 0, 0.5); //Nitsan: x
	else
		Desired_Pos.set(0, 0.2, 0.5); //Nitsan: y


	cVector3d pos;
	hapticDevice->getPosition(pos);
	cVector3d velocity;
	hapticDevice->getLinearVelocity(velocity);
	cVector3d force_toPos;
	force_toPos.zero();
	cVector3d force22;
	force22.zero();


	//double k = 15 * 0.001;
	//double b = 15 * 0.001;

	k *= 0.001;
	b *= 0.001;

	if (!direction) { //Nitsan: when we are using the y direction configuration, we need it to be a little more storng
		k *= 1.25;
		b *= 1.25;
	}


	double velX = velocity.x() / 0.01;
	double velY = velocity.y() / 0.01;
	double velZ = velocity.z() / 0.01;
	double Px = pos.x() / 0.01;
	double Py = pos.y() / 0.01;
	double Pz = pos.z() / 0.01;
	double counter = 0;
	double countmj = 0;

	double Kt;
	double dur = 2.5;
	double tmj;

	while (!gotToPosition) { //while got to position is still false-> meaning we havent gotten to the desired position yet
		countmj++;
		tmj = countmj / freqCounterHaptics.getFrequency();
		Kt = 10 * pow((tmj / dur), 3) - 15 * pow((tmj / dur), 4) + 6 * pow((tmj / dur), 5);
		if (Kt > 1)
		{
			Kt = 1;
		}
		//k *= Kt;
		if (direction)
			force_toPos.set(-k*(Px - Desired_Pos.x() / 0.01) - b*(velX), -k*(Py - Desired_Pos.y() / 0.01) - b*(velY), -k*(Pz - Desired_Pos.z() / 0.01) - b*(velZ));
		else
			force_toPos.set(0.0, -k*(Py - Desired_Pos.y() / 0.01) - b*(velY), -k*(Pz - Desired_Pos.z() / 0.01) - b*(velZ));
		hapticDevice->setForce(force_toPos);
		hapticDevice2->setForce(force22);
		//cout << abs(Px - Desired_Pos.x() / 0.01) << endl;

		if (direction) {
			if (abs(Px - Desired_Pos.x() / 0.01) < 100)
				counter++;
			if (counter == 1500)
				gotToPosition = true;
		}

		else {
			if (abs(Py - Desired_Pos.y() / 0.01) < 100)
				counter++;
			if (counter == 1500)
				gotToPosition = true;
		}
	}

	force_toPos.zero();
	hapticDevice->setForce(force_toPos);

	return gotToPosition;

}

// ----------------------------------------------------------------------

void ReadProtocolFile(bool test){
	//ThicknessLayer[][]
	int i;
	int j;
	char temp[200];
	FILE* ProtocolFile;


	if (test) {
		if (!(ProtocolFile = fopen("Protocol_Layer_Thickness_Test.txt", "r")))
			cout << "File doesn't exist" << endl;
	}
	else {
		if (!(ProtocolFile = fopen("Protocol_Layer_Thickness_Control.txt", "r")))
			cout << "File doesn't exist" << endl;
	}

	for (i = 0; i < NumTrials; i++) {
		for (j = 0; j < 9; j++){
			fscanf(ProtocolFile, "%s", temp);
			ThicknessLayer[i][j] = std::atof(temp);
			//cout << ThicknessLayer[i][j] << " ";
		}
		//cout << endl;
	}

	fclose(ProtocolFile);
	/*
	// checking
	FILE* ProtocolFile2 = fopen("Check.txt", "wt");

	for (i = 0; i < NumTrials; i++) { //Nitsan: going through every line
		for (j = 0; j < 6; j++) { //Nitsan: going through every component in the line
			fprintf(ProtocolFile2, "%.5f ", ThicknessLayer[i][j]);
		}
		//Nitsan: when we finish a line, we need do go down a line
		fprintf(ProtocolFile2, "\n");
	}
	
	fclose(ProtocolFile2);
	*/
}


// ---------------------------------------------
void getThickness(){


	int i;
	int k;
	double sum;

	cout << "Weight of patient: " << weight[(int)TrialNum - 1] << " kg" << endl;

	for (i = 0; i < 9; i++) { // 9 layers
		sum = 0;
		for (k = 0; k <= i; k++) { //summing for each
			sum += ThicknessLayer[(int)TrialNum - 1][k];
		}
		thick[i] = sum;
		//cout << thick[i] << " ";
	}
	//cout << endl;
}


// -----------------------------------------------

void ReadScaling(bool test) {
	int i;
	char temp[200];
	FILE* ProtocolFile;


	if (test) {
		if (!(ProtocolFile = fopen("Protocol_Scaling_Test.txt", "r")))
			cout << "File doesn't exist" << endl;
	}
	else {
		if (!(ProtocolFile = fopen("Protocol_Scaling_Control.txt", "r")))
			cout << "File doesn't exist" << endl;
	}

	for (i = 0; i < NumTrials; i++) {
			fscanf(ProtocolFile, "%s", temp);
			scaling_coef[i] = std::atof(temp);
			//cout << scaling_coef[i] << endl;
	}

	fclose(ProtocolFile);

}

//----------------------------------------------
void ReadWeight(bool test) {
	int i;
	char temp[200];
	FILE* ProtocolFile;


	if (test) {
		if (!(ProtocolFile = fopen("Protocol_Weight_Test.txt", "r")))
			cout << "File doesn't exist" << endl;
	}
	else {
		if (!(ProtocolFile = fopen("Protocol_Weight_Control.txt", "r")))
			cout << "File doesn't exist" << endl;
	}

	for (i = 0; i < NumTrials; i++) {
		fscanf(ProtocolFile, "%s", temp);
		weight[i] = std::atof(temp);
		//cout << "Weight of patient: " << weight[i] << " kg" << endl;
	}

	fclose(ProtocolFile);

}

//-------------------------------------------------------

void createPertubation(bool firstTimePert){
	cVector3d P;
	hapticDevice->getPosition(P);
	cVector3d desired_pos;
	desired_pos.set(0.25,0.25,0.25);
	cVector3d pertubation;
	double k = 5;
	double counter = 0;

	if (firstTimePert) {

		if (TrialNum == 77) {
				pertubation.set(-k*(P.x() - desired_pos.x()), 0, 0);
		}
		if (TrialNum == 78) {
			//for (counter = 0; counter < 2000; counter++)
				pertubation.set(0, 0, -k*(P.z() - desired_pos.z()));
		}
		if (TrialNum == 79) {
			pertubation.set(k*(P.x() - desired_pos.x()), 0, 0);
		}
		if (TrialNum == 80) {
			pertubation.set(0, 0, k*(P.z() - desired_pos.z()));
		}

		hapticDevice->setForce(pertubation);
	}

}
//---------------------------------------------------------------
void StopTrial() {
	double success;
	double failed_epidural;
	if (inES == 0) { //Nitsan: we are in the ES
		success = 1;
		failed_epidural = 0;
		//if (TrialNum <= 10) {
			cout << "Success" << endl;
		//}
	}
	if (inES == -1) { //Nitsan: before (failed epidural)
		success = 0;
		failed_epidural = 1;
		//if (TrialNum <= 10) {
			cout << "Failed Epidural" << endl;
		//}
	}
	//if (inES == 1 || dural_puncture) { //Nitsan: after (dural puncture)
	if (inES == 1) {
		success = 0;
		failed_epidural = 0;
		//if (TrialNum <= 10) {
			cout << "Dural Puncture" << endl;
		//}
	}
	string s("999");

	if (s.compare(Subject_Name)) {
		fprintf(Success_file, "%.0f                         %.0f                                 %.0f                                 %.0f\n", TrialNum, (double)dural_puncture, failed_epidural, success);
		//fclose(Success_file); //Nitsan: I added this		


	}
	injection = true;
}

// -------------------------------------------------------------
cVector3d createBuldges(double posZ){
	double thickness = 0.001;
	int i;
	cVector3d added_force;
	for (i = 1; i < 2; i+=2){
		if (posZ<thickness*(i + 1) && posZ>thickness*i){
			added_force.set(0.0,0.001*(posZ-thickness*i),0.0);
		}
	}
	return added_force;
}