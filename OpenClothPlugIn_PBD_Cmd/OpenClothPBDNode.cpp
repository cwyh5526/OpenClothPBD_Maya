#include "OpenClothPBDNode.h"

#define USE_TRIANGLE_BENDING_CONSTRAINT
#define PI 3.1415926536f
#define EPSILON  0.0000001f

struct DistanceConstraint { int p1, p2;	float rest_length, k; float k_prime; };
#ifdef USE_TRIANGLE_BENDING_CONSTRAINT
struct BendingConstraint { int p1, p2, p3;	float rest_length, w, k; float k_prime; };
#else
struct BendingConstraint { int p1, p2, p3, p4;	float rest_length1, rest_length2, w1, w2, k; float k_prime; };
#endif
//
static int prevNumX = 0, prevNumY = 0;
int prevcolorDim = 0;//color
static int cd;//color
static int numX = 0, numY = 0;
size_t total_points = (numX + 1)*(numY + 1);
float size = 20.0;//int size=20; //20170719
float hsize = size / 2.0f;

double fps = 0;
int countfps = 0;
float timeStep = 1.0f / 120.0f; //1.0/60.0f;
//float currentTime = 0;
double currentTime = 0;
double accumulator = timeStep;
int selected_index = -1;
float global_dampening = 0.98f; //DevO: 24.07.2011  //global velocity dampening !!!

std::vector<unsigned short> indices;
std::vector<DistanceConstraint> d_constraints;
std::vector<BendingConstraint> b_constraints;


//particle system
std::vector<glm::vec3> X; //position
std::vector<glm::vec3> tmp_X; //predicted position
//position constraint

int collisionGridSize;
std::vector<glm::vec3> V; //velocity
std::vector<glm::vec3> F;
std::vector<float> W; //inverse particle mass 
std::vector<glm::vec3> Ri; //Ri = Xi-Xcm 

//strain configuration
std::vector<float> Strain; //max deformation length 171011
float init_X;
float init_Y;//20171101

int oldX = 0, oldY = 0;
float rX = 15, rY = 0;
int state = 1;
float dist = -23;
const int GRID_SIZE = 10;

const size_t solver_iterations = 2; //number of solver iterations per step. PBD  

float kBend = 0.01f;//0.5f
float kStretch = 0.25f;//0.25f
float kDamp = 0.00125f;
glm::vec3 gravity = glm::vec3(0.0f, -0.00981f, 0.0f);
glm::vec3 nabi = glm::vec3(0.01f, 0.0f, 0.0f);//20170724

float mass = 1.f / total_points;//50.f / (total_points);//1.f/totalpoints

double frameTimeQP = 0;
float frameTime = 0;


glm::vec3 Up = glm::vec3(0, 1, 0), Right, viewDir;
float startTime = 0;// fps = 0;
int totalFrames = 0;

int numEllip = 420;// 400;
glm::mat4 ellipsoid[420];// 1];
glm::mat4 inverse_ellipsoid[420];// 1]; //20170925

int iStacks = 30;
int iSlices = 30;
float fRadius = 0.5f;
// Resolve constraint in object space
glm::vec3 center = glm::vec3(0, 0, 0); //object space center of ellipsoid
float radius =1.0f;
float radiusArr[420] = { 1.0f, };
float curvature[420] = { 2.0f, };
float heightDistance[420] = { 0.f, };

int actuatorIndex[420];//index of actuator
const int numActuatorX = 28;
const int numActuatorY = 15;
double heightArray[420] = //  1    2    3    4     5     6     7     8     9      10   11   12     13   14    15    16     17    18    19   20
{
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6, 1.6, 1.6, 1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //1

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6, 2.5, 3.8, 4.0, 3.8, 2.5, 1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     //2

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6, 3.1, 3.5, 4.2, 4.2, 3.5, 3.1, 1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //3

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 3.2, 3.6, 4.3, 4.3, 4.3, 3.6, 3.2, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     //4

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 3.4, 3.6, 4.3, 4.4, 4.4, 4.3, 3.6, 3.4, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //5

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.9, 3.5, 3.7, 5.5, 6.2, 5.5, 3.7, 3.5, 2.9, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     //6

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 3.4, 3.8, 3.8, 5.2, 5.2, 3.8, 3.8, 3.4, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //7

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 3.1, 3.7, 3.2, 3.7, 4.8, 3.7, 3.2, 3.7, 3.1, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     //8

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.8, 3.6, 3.0, 3.0, 4.3, 4.3, 3.0, 3.0, 3.6, 2.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //9

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 3.1, 4.1, 4.2, 4.5, 4.8, 4.5, 4.2, 4.1, 3.1, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     //10

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 4.0, 4.4, 4.6, 4.7, 4.7, 4.6, 4.4, 4.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //11

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6, 3.0, 3.6, 3.9, 4.0, 4.2, 4.0, 3.9, 3.6, 3.0, 1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     //12

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6, 2.8, 3.2, 3.6, 3.8, 3.8, 3.6, 3.2, 2.8, 1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //13

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6, 2.8, 3.0, 3.2, 3.4, 3.2, 3.0, 2.8, 1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     //14

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  //15
};

MObject OpenClothPBDNode::time;
MObject OpenClothPBDNode::inputMesh;
MObject OpenClothPBDNode::outputMesh;
MTypeId OpenClothPBDNode::id(0x80001);

MObject OpenClothPBDNode::height; 	//2017.07.19
MObject OpenClothPBDNode::width;
MObject OpenClothPBDNode::subWidth;
MObject OpenClothPBDNode::subHeight;
MObject OpenClothPBDNode::colorDim;//color

MFloatPointArray  OpenClothPBDNode::iarr;

MStatus returnStatus;

#define McheckErr(stat,msg)			\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
									}
//using namespace std;

void* OpenClothPBDNode::creator()
{
	return new OpenClothPBDNode;
}

MStatus OpenClothPBDNode::initialize()
{
	//define  the attributes of node (input/output)
	MStatus stat;
	MFnNumericAttribute nAttr;
	MFnUnitAttribute unitAttr;
	MFnTypedAttribute typedAttr;

	//cout << "OpenClothPBDNode Initialization" << endl;

	time = unitAttr.create("time", "tm", MFnUnitAttribute::kTime, 0.0, &stat);
	McheckErr(returnStatus, "ERROR creating OpenClothPBDNode time attribute\n");
	//MGlobal::displayInfo("######OpenClothPBDNode initialize()");

	inputMesh = typedAttr.create("inputMesh", "in", MFnData::kMesh, &stat);
	if (!stat)
		return stat;

	outputMesh = typedAttr.create("outputMesh", "out", MFnData::kMesh, &stat);
	if (!stat)
		return stat;

	if (!stat)
		return stat;

	typedAttr.setStorable(false);

	//2017.07.19
	width = nAttr.create("width", "w", MFnNumericData::kDouble, 0.0);
	height = nAttr.create("height", "h", MFnNumericData::kDouble, 0.0);
	subWidth = nAttr.create("subWidth", "sw", MFnNumericData::kInt, 0);
	subHeight = nAttr.create("subHeight", "sh", MFnNumericData::kInt, 0);
	colorDim = nAttr.create("colorDimension", "cd", MFnNumericData::kInt, 0);//color

	addAttribute(time);
	addAttribute(inputMesh);
	addAttribute(outputMesh);

	addAttribute(width);//2017.07.19
	addAttribute(height);//2017.07.19
	addAttribute(subWidth);//2017.07.19
	addAttribute(subHeight);//2017.07.19
	addAttribute(colorDim);//color

	attributeAffects(inputMesh, outputMesh);
	attributeAffects(time, outputMesh);

	attributeAffects(width, outputMesh);//2017.07.19
	attributeAffects(height, outputMesh);//2017.07.19
	attributeAffects(subWidth, outputMesh);//2017.07.19
	attributeAffects(subHeight, outputMesh);//2017.07.19

	attributeAffects(colorDim, outputMesh);//2017.07.19

	return MS::kSuccess;
}

MStatus OpenClothPBDNode::compute(const MPlug& plug, MDataBlock& data)

{
	MStatus returnStatus;
	if (plug == outputMesh)
	{
		MGlobal::displayInfo("######OpenClothPBDNode compute()");
		/* Get time */
		MDataHandle timeData = data.inputValue(time, &returnStatus);
		McheckErr(returnStatus, "Error getting time data handle\n");
		MTime timeD = timeData.asTime();


		/* Get width, height, subWidth, SubHeight */ //20170719
		MDataHandle wHnd = data.inputValue(width, &returnStatus);
		McheckErr(returnStatus, "Error getting width data handle\n");
		MDataHandle hHnd = data.inputValue(height, &returnStatus);
		McheckErr(returnStatus, "Error getting height data handle\n");
		size = wHnd.asFloat();
		hsize = hHnd.asFloat();

		MDataHandle subWHnd = data.inputValue(subWidth, &returnStatus);
		McheckErr(returnStatus, "Error getting subWidth data handle\n");
		MDataHandle subHHnd = data.inputValue(subHeight, &returnStatus);
		McheckErr(returnStatus, "Error getting subHeight data handle\n");

		MDataHandle cdHnd = data.inputValue(colorDim, &returnStatus);//color
		McheckErr(returnStatus, "Error getting subHeight data handle\n");//color

		prevNumX = numX;//20170724
		prevNumY = numY;//20170724

		prevcolorDim = cd;

		numX = subWHnd.asInt();
		numY = subHHnd.asInt();
		total_points = (numX + 1)*(numY + 1);

		cd = cdHnd.asInt();

		/* Get input*/
		MDataHandle inputData = data.inputValue(inputMesh, &returnStatus);
		if (!returnStatus) return returnStatus;
		else
		{

			MObject planeObj = inputData.asMesh();//handler가 mesh로 만들어줌.
			MFnMesh meshFn(planeObj, &returnStatus);//pass on to an FnMesh function set to operate on
			if (!returnStatus)	return returnStatus;
			else
			{

				//second data handle
				MDataHandle clothHandle = data.outputValue(outputMesh, &returnStatus);
				if (!returnStatus) return returnStatus;

				//create data obj to pass through the dependency graph and not a DAG object.
				MFnMeshData dataCreator;
				MObject newClothDataWraaper = dataCreator.create(&returnStatus);
				if (!returnStatus) return returnStatus;

				// user-writtencode
				createCloth(timeD, planeObj, newClothDataWraaper, returnStatus);
				if (!returnStatus) return returnStatus;

				//add new cloth to the data block so that the output changes
				clothHandle.set(newClothDataWraaper);

				returnStatus = data.setClean(plug);
				if (!returnStatus) return returnStatus;
			}
		}
		data.setClean(plug);

	}
	else
	{
		cerr << "unknow plug\n";
		return MS::kUnknownParameter;
	}
	return MS::kSuccess;
}

MObject OpenClothPBDNode::createCloth(const MTime& time, MObject& inData, MObject& outData, MStatus& stat)
{
	//MGlobal::displayInfo("######OpenClothPBDNode create()");
	MFnMesh meshFn;
	meshFn.copy(inData, outData);
	meshFn.setObject(outData);

	returnStatus = meshFn.getPoints(iarr, MSpace::kObject);
	if (returnStatus != MS::kSuccess){
		cerr << "Error in getting Vertices:" << returnStatus << endl;
		return MObject::kNullObj;
	}

	//meshFn.assignColors(colorIds, NULL);
	//meshFn.setDisplayColors(true);

	MColorArray colors;
	MIntArray vertexId;
	//MIntArray colorIds;

	static bool first = true; //20170721
	if (time.value() == 1){
		first = true;
	}
	if (prevNumX != numX || prevNumY != numY || prevcolorDim != cd) //20170817
	{
		first = true;
	}
	if (first)
	{
		InitializeOpenCloth();

		colors.clear();
		vertexId.clear();

		for (int i = 0; i<total_points; i++)
		{
			colors.append(0.35f, 0.35f, 0.35f, 1.0f);
			vertexId.append(i);
		}

		meshFn.setVertexColors(colors, vertexId);

		meshFn.setPoints(iarr);

		first = false;
	}
	else {
		StepPhysics(timeStep);

		if (cd == 1)
		{
			//MGlobal::displayInfo("ㅎㅇㅎㅇ");
			for (int i = 0; i<numActuatorX*numActuatorY; i++)
			{
				double s = (1 - std::min(std::max(Strain[actuatorIndex[i]], 0.f), 1.f)) * 241;
				MColor c(MColor::kHSV, s, 1.0, 1.0, 1.0);
				colors.append(c);
				vertexId.append(actuatorIndex[i]);

			}
			meshFn.setVertexColors(colors, vertexId);
			meshFn.setPoints(iarr);
		}
		else if (cd == 2)
		{
			for (int i = 0; i<total_points; i++)
			{
				double s = (1 - std::min(std::max(Strain[i], 0.f), 1.f)) * 241;
				//MGlobal::displayInfo(MString("strain = ") + Strain[i]);
				MColor c(MColor::kHSV, s, 1.0, 1.0, 1.0);
				colors.append(c);
				vertexId.append(i);

			}
			meshFn.setVertexColors(colors, vertexId);
			meshFn.setPoints(iarr);
		}
		else if (cd == 3)
		{
			if (curvature[0] != 0){
				for (int i = 0; i < total_points; i++)
				{
					double s = (1 - std::min(std::max(radiusArr[i], 0.f), 1.f)) * 241;
					//MGlobal::displayInfo(MString("strain = ") + Strain[i]);
					MColor c(MColor::kHSV, s, 1.0, 1.0, 1.0);
					colors.append(c);
					vertexId.append(i);

				}
				meshFn.setVertexColors(colors, vertexId);
				meshFn.setPoints(iarr);
			}
		}
		else
		{
			meshFn.setPoints(iarr);
		}
	}
	return outData;
}

void AddDistanceConstraint(int a, int b, float k) {
	DistanceConstraint c;
	c.p1 = a;
	c.p2 = b;
	c.k = k;
	c.k_prime = 1.0f - pow((1.0f - c.k), 1.0f / solver_iterations);  //1.0f-pow((1.0f-c.k), 1.0f/ns);

	if (c.k_prime>1.0)
		c.k_prime = 1.0;

	glm::vec3 deltaP = X[c.p1] - X[c.p2];
	c.rest_length = glm::length(deltaP);

	d_constraints.push_back(c);
}

#ifdef USE_TRIANGLE_BENDING_CONSTRAINT
void AddBendingConstraint(int pa, int pb, int pc, float k) {
	BendingConstraint c;
	c.p1 = pa;
	c.p2 = pb;
	c.p3 = pc;

	c.w = W[pa] + W[pb] + 2 * W[pc];
	glm::vec3 center = 0.3333f * (X[pa] + X[pb] + X[pc]);
	c.rest_length = glm::length(X[pc] - center);
	c.k = k;
	c.k_prime = 1.0f - pow((1.0f - c.k), 1.0f / solver_iterations);  //1.0f-pow((1.0f-c.k), 1.0f/ns);
	if (c.k_prime>1.0)
		c.k_prime = 1.0;
	b_constraints.push_back(c);
}
#else
void AddBendingConstraint(int pa, int pb, int pc, int pd, float k) {
	BendingConstraint c;
	c.p1 = pa;
	c.p2 = pb;
	c.p3 = pc;
	c.p4 = pd;
	c.w1 = W[pa] + W[pb] + 2 * W[pc];
	c.w2 = W[pa] + W[pb] + 2 * W[pd];
	glm::vec3 center1 = 0.3333f * (X[pa] + X[pb] + X[pc]);
	glm::vec3 center2 = 0.3333f * (X[pa] + X[pb] + X[pd]);
	c.rest_length1 = glm::length(X[pc] - center1);
	c.rest_length2 = glm::length(X[pd] - center2);
	c.k = k;

	c.k_prime = 1.0f - pow((1.0f - c.k), 1.0f / solver_iterations);  //1.0f-pow((1.0f-c.k), 1.0f/ns);
	if (c.k_prime>1.0)
		c.k_prime = 1.0;
	b_constraints.push_back(c);
}
#endif
#ifndef USE_TRIANGLE_BENDING_CONSTRAINT
inline float GetDihedralAngle(BendingConstraint c, float& d, glm::vec3& n1, glm::vec3& n2) {
	n1 = GetNormal(c.p1, c.p2, c.p3);
	n2 = GetNormal(c.p1, c.p2, c.p4);
	d = glm::dot(n1, n2);
	return acos(d);
}
#else
inline int getIndex(int i, int j) {
	return j*(numX + 1) + i;
}
#endif

void OpenClothPBDNode::InitializeOpenCloth()
{
	//Initialize variable and Cosntraints
	//MGlobal::displayInfo("######OpenClothPBDNode InitializeOpenCloth()");

	size_t i = 0, j = 0, count = 0;
	int l1 = 0, l2 = 0;
	float ypos = 7.0f;
	int v = numY + 1;
	int u = numX + 1;
	for (int i = 0; i < 420; ++i){
		if (radiusArr[i] != 1.f){
			MGlobal::displayInfo("######OpenClothPBDNode InitializeOpenCloth()");
			break;
		}
	}
	//iarr.clear();//20170601 //20170901

	indices.resize(numX*numY * 2 * 3);

	X.resize(total_points);
	tmp_X.resize(total_points);
	Strain.resize(total_points);
	V.resize(total_points);
	F.resize(total_points);
	Ri.resize(total_points);

	//fill in positions
	for (int i = 0; i < total_points; i++) {
		X[i].x = iarr[i].x;
		X[i].y = iarr[i].y;
		X[i].z = iarr[i].z;
	}

	//strain config init
	init_X = sqrt(pow(X[0].x - X[1].x, 2) +
		pow(X[0].y - X[1].y, 2) +
		pow(X[0].z - X[1].z, 2));
	init_Y = sqrt(pow(X[0].x - X[getIndex(1, 0)].x, 2) +
		pow(X[0].y - X[getIndex(0, 1)].y, 2) +
		pow(X[0].z - X[getIndex(0, 1)].z, 2)); //?20171103

	for (int i = 0; i < total_points; i++) {
		Strain[i] = 0;
	}

	collisionGridSize = (int)(numX / numActuatorX);

	//fill in actuator indices 20171027
	int unitGridX = (int)(numX) / (numActuatorX * 2);//grid numbers between actuators.
	int unitGridY = (int)(numY) / (numActuatorY * 2);
	int x, y;

	for (j = 0; j < numActuatorY; j++)
	{
		for (i = 0; i < numActuatorX; i++)
		{
			x = unitGridX*i; //
			y = unitGridY*j;
			if (j % 2 == 0) {
				actuatorIndex[i + j * numActuatorX] = getIndex(2 * x + 2 * unitGridX, 2 * y + unitGridY);//cloth grid index of Layer1 actuator
			}
			else {
				actuatorIndex[i + j * numActuatorX] = getIndex(2 * x + unitGridX, 2 * y + unitGridY);//cloth grid index of Layer2 actuator
			}
			radiusArr[i + j * numActuatorX] = 1.f;//20171104
		}
	}

	///DevO: 24.07.2011
	W.resize(total_points);
	for (i = 0; i<total_points; i++) {
		W[i] = 1.0f / mass;
	}
	/// 2 Fixed Points 
	//W[0] = 0.0;
	//W[numX] = 0.0;
	//W[total_points - 1] = 0.0; //20170724
	//W[total_points - numX] = 0.0;//20170724

	for (i = 0; i <= numX; i++){//20170529 한줄 다 20170911 아래 위 다 //index 수정 20170921
		W[i] = 0.0;//맨아래줄
		W[numY*(numX + 1) + i] = 0.0;//맨 윗줄

	}
	for (int j = 0; j <= numY; j++){
		W[j*(numX + 1)] = 0.0; //왼쪽
		W[(j + 1)*(numX + 1) - 1] = 0.0;//오른쪽
	}

	memcpy(&tmp_X[0].x, &X[0].x, sizeof(glm::vec3)*total_points);

	//fill in velocities	 
	memset(&(V[0].x), 0, total_points*sizeof(glm::vec3));


	//fill in indices
	unsigned short* id = &indices[0];
	for (int i = 0; i < numY; i++) {
		for (int j = 0; j < numX; j++) {
			int i0 = i * (numX + 1) + j;
			int i1 = i0 + 1;
			int i2 = i0 + (numX + 1);
			int i3 = i2 + 1;
			if ((j + i) % 2) {
				*id++ = i0; *id++ = i2; *id++ = i1;
				*id++ = i1; *id++ = i2; *id++ = i3;
			}
			else {
				*id++ = i0; *id++ = i2; *id++ = i3;
				*id++ = i0; *id++ = i3; *id++ = i1;
			}
		}
	}

	//check the damping values
	if (kStretch>1)
		kStretch = 1;
	if (kStretch<0)
		kStretch = 0;
	if (kBend>1)
		kBend = 1;
	if (kBend<0)
		kBend = 0;
	if (kDamp>1)
		kDamp = 1;
	if (kDamp<0)
		kDamp = 0;
	if (global_dampening>1)
		global_dampening = 1;

	//setup constraints


	//20170901 d_constraints should be initialized too.
	d_constraints.clear();
	b_constraints.clear();


	// Horizontal
	for (l1 = 0; l1 < v; l1++)	// v
		for (l2 = 0; l2 < (u - 1); l2++) {
			AddDistanceConstraint((l1 * u) + l2, (l1 * u) + l2 + 1, kStretch);
		}

	// Vertical
	for (l1 = 0; l1 < (u); l1++)
		for (l2 = 0; l2 < (v - 1); l2++) {
			AddDistanceConstraint((l2 * u) + l1, ((l2 + 1) * u) + l1, kStretch);
		}


	// Shearing distance constraint
	for (l1 = 0; l1 < (v - 1); l1++)
		for (l2 = 0; l2 < (u - 1); l2++) {
			AddDistanceConstraint((l1 * u) + l2, ((l1 + 1) * u) + l2 + 1, kStretch);
			AddDistanceConstraint(((l1 + 1) * u) + l2, (l1 * u) + l2 + 1, kStretch);
		}


	// create bending constraints	
#ifdef USE_TRIANGLE_BENDING_CONSTRAINT
	//add vertical constraints
	for (int i = 0; i <= numX; i++) {
		for (int j = 0; j<numY - 1; j++) {
			AddBendingConstraint(getIndex(i, j), getIndex(i, (j + 1)), getIndex(i, j + 2), kBend);
		}
	}
	//add horizontal constraints
	for (int i = 0; i<numX - 1; i++) {
		for (int j = 0; j <= numY; j++) {
			AddBendingConstraint(getIndex(i, j), getIndex(i + 1, j), getIndex(i + 2, j), kBend);
		}
	}

#else
	for (int i = 0; i < v - 1; ++i) {
		for (int j = 0; j < u - 1; ++j) {
			int p1 = i * (numX + 1) + j;
			int p2 = p1 + 1;
			int p3 = p1 + (numX + 1);
			int p4 = p3 + 1;

			if ((j + i) % 2) {
				AddBendingConstraint(p3, p2, p1, p4, kBend);
			}
			else {
				AddBendingConstraint(p4, p1, p3, p2, kBend);
			}
		}
	}
	float d;
	glm::vec3 n1, n2;
	phi0.resize(b_constraints.size());

	for (i = 0; i<b_constraints.size(); i++) {
		phi0[i] = GetDihedralAngle(b_constraints[i], d, n1, n2);
	}
#endif

	//create a basic ellipsoid object
	/*400 */
	for (int i = 0; i < numActuatorX*numActuatorY; i++)
	{
		ellipsoid[i] = glm::translate(glm::mat4(1), glm::vec3(0 - radiusArr[i], tmp_X[actuatorIndex[i]].y, tmp_X[actuatorIndex[i]].z));

		inverse_ellipsoid[i] = glm::inverse(ellipsoid[i]);
	}

	/* cloth ize 20*20
	for (int j = 0; j < 20; j++){
	for (int i = 0; i <20; i++){
	ellipsoid[i + 20 * j] = glm::translate(glm::mat4(1), glm::vec3(0 - radius, i - 9.5, -9.5 + j ));
	inverse_ellipsoid[i + 20 * j] = glm::inverse(ellipsoid[i + 20 * j]);
	}
	}*/
	//ellipsoid[0] = glm::translate(glm::mat4(1), glm::vec3(0 - radius, 0,0));
	//inverse_ellipsoid[0] = glm::inverse(ellipsoid[0]);

	//ellipsoid = glm::translate(glm::mat4(1), glm::vec3(-1, 0, 0));
	//ellipsoid = glm::rotate(ellipsoid, 45.0f, glm::vec3(1, 0, 0));
	//ellipsoid = glm::scale(ellipsoid, glm::vec3(fRadius, fRadius, fRadius / 2));
	//inverse_ellipsoid = glm::inverse(ellipsoid);



}
void OpenClothPBDNode::StepPhysics(float dt){

	//MGlobal::displayInfo("######OpenClothPBDNode StepPhy()");
	ComputeForces();
	IntegrateExplicitWithDamping(dt);

	// for collision constraints
	UpdateInternalConstraints(dt);
	UpdateExternalConstraints();

	Integrate(dt);

	if (cd == 1)
	{
		CalActuatorStrain();
	}
	else if (cd == 2)
	{
		CalNodalStrain();
	}
}

void OpenClothPBDNode::ComputeForces() {
	size_t i = 0;

	for (i = 0; i<total_points; i++) {
		F[i] = glm::vec3(0);

		//add gravity force
		//if (W[i]>0)
		//F[i] += gravity;
		//F[total_points / 2] += nabi;
	}

	//F[total_points / 2] += glm::vec3(100, 0, 0);//20170911 10의 힘을 z 방향으로 준다
	//F[total_points / 4] += glm::vec3(100, 0, 0);//20170911 10의 힘을 z 방향으로 준다
	//F[total_points-(total_points/ 4)] += glm::vec3(0, 0, 100.0);//20170911 10의 힘을 z 방향으로 준다
}

void OpenClothPBDNode::IntegrateExplicitWithDamping(float deltaTime) {
	float deltaTimeMass = deltaTime;
	size_t i = 0;

	glm::vec3 Xcm = glm::vec3(0);
	glm::vec3 Vcm = glm::vec3(0);
	float sumM = 0;
	for (i = 0; i<total_points; i++) {

		V[i] *= global_dampening; //global velocity dampening !!!		
		V[i] = V[i] + (F[i] * deltaTime)*W[i];

		//calculate the center of mass's position 
		//and velocity for damping calc
		Xcm += (X[i] * mass);
		Vcm += (V[i] * mass);
		sumM += mass;
	}
	Xcm /= sumM;
	Vcm /= sumM;

	glm::mat3 I = glm::mat3(1);
	glm::vec3 L = glm::vec3(0);
	glm::vec3 w = glm::vec3(0);//angular velocity


	for (i = 0; i<total_points; i++) {
		Ri[i] = (X[i] - Xcm);

		L += glm::cross(Ri[i], mass*V[i]);

		//thanks to DevO for pointing this and these notes really helped.
		//http://www.sccg.sk/~onderik/phd/ca2010/ca10_lesson11.pdf

		glm::mat3 tmp = glm::mat3(0, -Ri[i].z, Ri[i].y,
			Ri[i].z, 0, -Ri[i].x,
			-Ri[i].y, Ri[i].x, 0);
		I += (tmp*glm::transpose(tmp))*mass;
	}

	w = glm::inverse(I)*L;

	//apply center of mass damping
	for (i = 0; i<total_points; i++) {
		glm::vec3 delVi = Vcm + glm::cross(w, Ri[i]) - V[i];
		V[i] += kDamp*delVi;
	}

	//calculate predicted position
	for (i = 0; i<total_points; i++) {
		if (W[i] <= 0.0) {
			tmp_X[i] = X[i]; //fixed points
		}
		else {
			tmp_X[i] = X[i] + (V[i] * deltaTime);
		}
	}
}

void OpenClothPBDNode::Integrate(float deltaTime) {
	float inv_dt = 1.0f / deltaTime;
	size_t i = 0;

	for (i = 0; i<total_points; i++) {
		V[i] = (tmp_X[i] - X[i])*inv_dt;
		X[i] = tmp_X[i];

		iarr[i].x = X[i].x;//20170615
		iarr[i].y = X[i].y;
		iarr[i].z = X[i].z;
	}
}

void OpenClothPBDNode::UpdateDistanceConstraint(int i) {

	DistanceConstraint c = d_constraints[i];
	glm::vec3 dir =
		tmp_X[
			c.p1
		] -
		tmp_X[
			c.p2
		];

			float len = glm::length(dir);
			if (len <= EPSILON)
				return;

			float w1 = W[c.p1];
			float w2 = W[c.p2];
			float invMass = w1 + w2;
			if (invMass <= EPSILON)
				return;

			glm::vec3 dP = (1.0f / invMass) * (len - c.rest_length) * (dir / len)* c.k_prime;
			if (w1 > 0.0)
				tmp_X[c.p1] -= dP*w1;

			if (w2 > 0.0)
				tmp_X[c.p2] += dP*w2;
}

void OpenClothPBDNode::UpdateBendingConstraint(int index) {
	size_t i = 0;
	BendingConstraint c = b_constraints[index];

#ifdef USE_TRIANGLE_BENDING_CONSTRAINT
	//Using the paper suggested by DevO
	//http://image.diku.dk/kenny/download/kelager.niebe.ea10.pdf

	//global_k is a percentage of the global dampening constant 
	float global_k = global_dampening*0.01f;
	glm::vec3 center = 0.3333f * (tmp_X[c.p1] + tmp_X[c.p2] + tmp_X[c.p3]);
	glm::vec3 dir_center = tmp_X[c.p3] - center;
	float dist_center = glm::length(dir_center);

	float diff = 1.0f - ((global_k + c.rest_length) / dist_center);
	glm::vec3 dir_force = dir_center * diff;
	glm::vec3 fa = c.k_prime * ((2.0f*W[c.p1]) / c.w) * dir_force;
	glm::vec3 fb = c.k_prime * ((2.0f*W[c.p2]) / c.w) * dir_force;
	glm::vec3 fc = -c.k_prime * ((4.0f*W[c.p3]) / c.w) * dir_force;

	if (W[c.p1] > 0.0)  {
		tmp_X[c.p1] += fa;
	}
	if (W[c.p2] > 0.0) {
		tmp_X[c.p2] += fb;
	}
	if (W[c.p3] > 0.0) {
		tmp_X[c.p3] += fc;
	}
#else

	//Using the dihedral angle approach of the position based dynamics		
	float d = 0, phi = 0, i_d = 0;
	glm::vec3 n1 = glm::vec3(0), n2 = glm::vec3(0);

	glm::vec3 p1 = tmp_X[c.p1];
	glm::vec3 p2 = tmp_X[c.p2] - p1;
	glm::vec3 p3 = tmp_X[c.p3] - p1;
	glm::vec3 p4 = tmp_X[c.p4] - p1;

	glm::vec3 p2p3 = glm::cross(p2, p3);
	glm::vec3 p2p4 = glm::cross(p2, p4);

	float lenp2p3 = glm::length(p2p3);

	if (lenp2p3 == 0.0) { return; } //need to handle this case.

	float lenp2p4 = glm::length(p2p4);

	if (lenp2p4 == 0.0) { return; } //need to handle this case.

	n1 = glm::normalize(p2p3);
	n2 = glm::normalize(p2p4);

	d = glm::dot(n1, n2);
	phi = acos(d);

	//try to catch invalid values that will return NaN.
	// sqrt(1 - (1.0001*1.0001)) = NaN 
	// sqrt(1 - (-1.0001*-1.0001)) = NaN 
	if (d<-1.0)
		d = -1.0;
	else if (d>1.0)
		d = 1.0; //d = clamp(d,-1.0,1.0);

	//in both case sqrt(1-d*d) will be zero and nothing will be done.
	//0?case, the triangles are facing in the opposite direction, folded together.
	if (d == -1.0){
		phi = PI;  //acos(-1.0) == PI
		if (phi == phi0[index])
			return; //nothing to do 

		//in this case one just need to push 
		//vertices 1 and 2 in n1 and n2 directions, 
		//so the constrain will do the work in second iterations.
		if (c.p1 != 0 && c.p1 != numX)
			tmp_X[c.p3] += n1 / 100.0f;

		if (c.p2 != 0 && c.p2 != numX)
			tmp_X[c.p4] += n2 / 100.0f;

		return;
	}
	if (d == 1.0){ //180?case, the triangles are planar
		phi = 0.0;  //acos(1.0) == 0.0
		if (phi == phi0[index])
			return; //nothing to do 
	}

	i_d = sqrt(1 - (d*d))*(phi - phi0[index]);

	glm::vec3 p2n1 = glm::cross(p2, n1);
	glm::vec3 p2n2 = glm::cross(p2, n2);
	glm::vec3 p3n2 = glm::cross(p3, n2);
	glm::vec3 p4n1 = glm::cross(p4, n1);
	glm::vec3 n1p2 = -p2n1;
	glm::vec3 n2p2 = -p2n2;
	glm::vec3 n1p3 = glm::cross(n1, p3);
	glm::vec3 n2p4 = glm::cross(n2, p4);

	glm::vec3 q3 = (p2n2 + n1p2*d) / lenp2p3;
	glm::vec3 q4 = (p2n1 + n2p2*d) / lenp2p4;
	glm::vec3 q2 = (-(p3n2 + n1p3*d) / lenp2p3) - ((p4n1 + n2p4*d) / lenp2p4);

	glm::vec3 q1 = -q2 - q3 - q4;

	float q1_len2 = glm::dot(q1, q1);// glm::length(q1)*glm::length(q1);
	float q2_len2 = glm::dot(q2, q2);// glm::length(q2)*glm::length(q1);
	float q3_len2 = glm::dot(q3, q3);// glm::length(q3)*glm::length(q1);
	float q4_len2 = glm::dot(q4, q4);// glm::length(q4)*glm::length(q1); 

	float sum = W[c.p1] * (q1_len2)+
		W[c.p2] * (q2_len2)+
		W[c.p3] * (q3_len2)+
		W[c.p4] * (q4_len2);

	glm::vec3 dP1 = -((W[c.p1] * i_d) / sum)*q1;
	glm::vec3 dP2 = -((W[c.p2] * i_d) / sum)*q2;
	glm::vec3 dP3 = -((W[c.p3] * i_d) / sum)*q3;
	glm::vec3 dP4 = -((W[c.p4] * i_d) / sum)*q4;

	if (W[c.p1] > 0.0) {
		tmp_X[c.p1] += dP1*c.k;
	}
	if (W[c.p2] > 0.0) {
		tmp_X[c.p2] += dP2*c.k;
	}
	if (W[c.p3] > 0.0) {
		tmp_X[c.p3] += dP3*c.k;
	}
	if (W[c.p4] > 0.0) {
		tmp_X[c.p4] += dP4*c.k;
	}
#endif
}
//----------------------------------------------------------------------------------------------------
void OpenClothPBDNode::GroundCollision() //DevO: 24.07.2011
{
	for (size_t i = 0; i<total_points; i++) {
		if (tmp_X[i].y<0) //collision with ground
			tmp_X[i].y = 0;
	}
}
void OpenClothPBDNode::EllipsoidCollision() {

	int numCollisionGridY ; //y-range of cloth grid for collision detection 
	int numCollisionGridX ; //x-range of cloth grid for collision dtetction

	
	int clothIndex;  //1-dimension index
	int clothIndexX; //2d index
	int clothIndexY; //2d index
	
	for (int e = 0; e < numActuatorX*numActuatorY;e++){			//for all ellipsoid e
		clothIndexX = actuatorIndex[e] % (numX + 1);			//calculate the x,y 
		clothIndexY = (int)(actuatorIndex[e] / (numX+1));

		numCollisionGridY = (int)(radiusArr[e] / init_Y);
		numCollisionGridX = (int)(radiusArr[e] / init_X);

		for (int y = std::max(0, clothIndexY - numCollisionGridY); y < std::min(clothIndexY + numCollisionGridY, numY); y++){
			for (int x = std::max(0, clothIndexX - numCollisionGridX); x < std::min(clothIndexX + numCollisionGridX, numX); x++){
				clothIndex = getIndex(x, y);

				glm::vec4 X_0 = (inverse_ellipsoid[e] * glm::vec4(tmp_X[clothIndex], 1));
				glm::vec3 delta0 = glm::vec3(X_0.x, X_0.y, X_0.z) - center;
				float distance = glm::length(delta0);
				if (distance <  radiusArr[e]) {
					delta0 = (radiusArr[e] - distance) * delta0 / distance;

					// Transform the delta back to original space
					glm::vec3 delta;
					glm::vec3 transformInv;
					transformInv = glm::vec3(ellipsoid[e][0].x, ellipsoid[e][1].x, ellipsoid[e][2].x);
					transformInv /= glm::dot(transformInv, transformInv);
					delta.x = glm::dot(delta0, transformInv);
					transformInv = glm::vec3(ellipsoid[e][0].y, ellipsoid[e][1].y, ellipsoid[e][2].y);
					transformInv /= glm::dot(transformInv, transformInv);
					delta.y = glm::dot(delta0, transformInv);
					transformInv = glm::vec3(ellipsoid[e][0].z, ellipsoid[e][1].z, ellipsoid[e][2].z);
					transformInv /= glm::dot(transformInv, transformInv);
					delta.z = glm::dot(delta0, transformInv);
					tmp_X[clothIndex] += delta;
					V[clothIndex] = glm::vec3(0);
				}
			}
		}
		
	}
	
}

void OpenClothPBDNode::EllipsoidMove(){
	/* 20170921 */
	
	
	//printf("ellipsoid[0]: %f\n", ellipsoid[3].x);
	for (int i = 0; i < numActuatorX*numActuatorY; i++) {
		
		//if (ellipsoid[i][3].x >= heightArray[i] - radiusArr[i]) {//limit
		//	ellipsoid[i] = glm::translate(glm::mat4(1), glm::vec3(heightArray[i] - radiusArr[i], ellipsoid[i][3].y, ellipsoid[i][3].z));//2017.05.29 glm::vec3(0,2,0))
		//	//ellipsoid[i] = glm::scale(ellipsoid[i], glm::vec3(radius, radius, radius));//20171103
		//	inverse_ellipsoid[i] = glm::inverse(ellipsoid[i]);

		//}
		//else {

			ellipsoid[i] = glm::translate(glm::mat4(1), glm::vec3(tmp_X[actuatorIndex[i]].x - radiusArr[i], ellipsoid[i][3].y, ellipsoid[i][3].z));//2017.05.29 glm::vec3(0,2,0))//20170921(-1+0.01*i)에서 ellipsoid[3].x + 0.01로 수정. i static이라 초기화 문제 때문. 
			//ellipsoid[i] = glm::scale(ellipsoid[i], glm::vec3(radius, radius, radius));//20171103

			inverse_ellipsoid[i] = glm::inverse(ellipsoid[i]);
		//}
	}


}
void OpenClothPBDNode::UpdatePositionConstraint() {

	for (int i = 0; i < numActuatorX*numActuatorY; i++) {

		if (tmp_X[actuatorIndex[i]].x >= heightArray[i]) {
			tmp_X[actuatorIndex[i]].x = heightArray[i];
			W[actuatorIndex[i]] = 0.0;
		}
		else {
			tmp_X[actuatorIndex[i]].x += (heightArray[i]) / 200;
			W[actuatorIndex[i]] = 0.0;
		}

	}
}

void OpenClothPBDNode::UpdateExternalConstraints() {
	
	CalEllipsoidRadius();

	EllipsoidMove();

	EllipsoidCollision();

}
//----------------------------------------------------------------------------------------------------
void OpenClothPBDNode::UpdateInternalConstraints(float deltaTime) {
	size_t i = 0;

	//printf(" UpdateInternalConstraints \n ");
	for (size_t si = 0; si<solver_iterations; ++si) {
		for (i = 0; i<d_constraints.size(); i++) {
			UpdateDistanceConstraint(i);
		}
		for (i = 0; i<b_constraints.size(); i++) {
			UpdateBendingConstraint(i);
		}
		UpdatePositionConstraint();
		//GroundCollision();
	}
}

//171011
//171012
void OpenClothPBDNode::CalActuatorStrain()
{
	//MGlobal::displayInfo("calStr");

	int point = 0;

	//actuator strain
	for (int i = 0; i < numActuatorX; i++)
	{
		for (int j = 0; j < numActuatorY; j++)
		{
			point = i + numActuatorX * j;
			if ((i != 0) && (i != numActuatorX - 1) && (j != 0) && (j != numActuatorY - 1))
			{
				float len1 = 0, len2 = 0, len3 = 0, len4 = 0;
				for (int sublen = 0; sublen < collisionGridSize; sublen++)
				{
					len1 += distance(tmp_X[actuatorIndex[point] - collisionGridSize + sublen], tmp_X[actuatorIndex[point] - collisionGridSize + sublen + 1]);

				}
				for (int sublen = 0; sublen > -collisionGridSize; sublen--)
				{
					len2 += distance(tmp_X[actuatorIndex[point] + collisionGridSize + sublen], tmp_X[actuatorIndex[point] + collisionGridSize + sublen - 1]);
				}
				for (int sublen = 0; sublen < collisionGridSize; sublen++)
				{
					len3 += distance(tmp_X[actuatorIndex[point] - (numX + 1) * (collisionGridSize - sublen)], tmp_X[actuatorIndex[point] - (numX + 1) * (collisionGridSize - sublen - 1)]);
				}
				for (int sublen = 0; sublen > -collisionGridSize; sublen--)
				{
					len4 += distance(tmp_X[actuatorIndex[point] + (numX + 1) * (collisionGridSize + sublen)], tmp_X[actuatorIndex[point] + (numX + 1) * (collisionGridSize + sublen - 1)]);
				}

				Strain[actuatorIndex[point]] = std::max(std::max(len1, len2), std::max(len3, len4));

				if (Strain[actuatorIndex[point]] >= init_X * collisionGridSize)
					Strain[actuatorIndex[point]] = (Strain[actuatorIndex[point]] - (init_X * collisionGridSize)) / (init_X * collisionGridSize);
				else
					Strain[actuatorIndex[point]] = ((init_X * collisionGridSize) - Strain[actuatorIndex[point]]) / (init_X * collisionGridSize);
			}
		}
	}
	//MGlobal::displayInfo(MString()+Strain[188]);
}

void OpenClothPBDNode::CalNodalStrain()
{
	int point = 0;

	//nodal strain
	for (int j = 0; j <= numY; j++)
	{
		for (int i = 0; i <= numX; i++)
		{
			point = i + (numX + 1)*j;

			if ((i != 0) && (i != numX) && (j != 0) && (j != numY))
			{
				float len1, len2, len3, len4;
				len1 = distance(tmp_X[point], tmp_X[point - (numX + 1)]);
				len2 = distance(tmp_X[point], tmp_X[point + (numX + 1)]);
				len3 = distance(tmp_X[point], tmp_X[point - 1]);
				len4 = distance(tmp_X[point], tmp_X[point + 1]);

				Strain[point] = std::max(std::max(len1, len2), std::max(len3, len4));

				if (Strain[point] >= init_X)
					Strain[point] = (Strain[point] - init_X) / (init_X);
				else
					Strain[point] = (init_X - Strain[point]) / (init_X);
			}			
		}
	}
}


void OpenClothPBDNode::CalEllipsoidRadius(){

	int c;
	int p[6]; //p0 is the centerindex of each points around 
	float disCP[6];
	float disPP[6];
	float angPP[6];
	float s[6];
	float a[6];
	float sumAng = 0.0f;
	float A = 0.0f;
	float deltaAng;


	for (int ay = 1; ay < numActuatorY - 1; ay++){ //for all actuators inside of the boundary
		for (int ax = 1; ax < numActuatorX-1; ax++){

			c = (ax)+(ay)*numActuatorX;

			if (ay % 2 == 0){//짝수 행, layer 1 27개 actuator
				p[0] = (ax)+(ay - 1)*numActuatorX; p[1] = (ax + 1) + (ay - 1)*numActuatorX;
				p[4] = (ax)+(ay + 1)*numActuatorX; p[5] = (ax + 1) + (ay + 1)*numActuatorX;
			}
			else{
				p[0] = (ax - 1) + (ay - 1)*numActuatorX; p[1] = (ax)+(ay - 1)*numActuatorX;
				p[4] = (ax - 1) + (ay + 1)*numActuatorX; p[5] = (ax)+(ay + 1)*numActuatorX;
			}
			p[2] = (ax - 1) + (ay)*numActuatorX;
			p[3] = (ax + 1) + (ay)*numActuatorX;

			/*height*/
			for (int i = 0; i < 6; i++){
				int dis = tmp_X[actuatorIndex[c]].x - tmp_X[actuatorIndex[p[i]]].x;// 양수면 c가 다른 애들보다 앞에 있다는 뜻임. 음수면 다들 나보다 앞에있다는 것. 그러면 크기가 별로 의미가 없나.
				if (heightDistance[c] < dis){ //가장 큰 distance를 넣어준다.
					heightDistance[c] = dis;
				}
			}
			/*curvature
			for (int i = 0; i < 6; i++)
			{
				disCP[i] = distance(tmp_X[actuatorIndex[c]], tmp_X[actuatorIndex[p[i]]]);
				disPP[i] = distance(tmp_X[actuatorIndex[p[i]]], tmp_X[actuatorIndex[p[(i + 1)%6]]]); //%6 하는건 disPP[5]는 p[5] p[0]이랑 해야되니까.
				
			}
			for (int i = 0; i < 6; i++){
				angPP[i] = acos((disCP[i] * disCP[i] + disCP[(i + 1) % 6] * disCP[(i + 1) % 6] - disPP[i] * disPP[i])/(2*disCP[i]*disCP[(i+1)%6]));
				s[i] = (disCP[i] + disCP[(i + 1) % 6] + disPP[i]) / 2;
				a[i] = sqrt(s[i] * (s[i] - disCP[i])*(s[i] - disCP[(i + 1) % 6])*(s[i] - disPP[i]));
				
			}
			for (int i = 0; i < 6; i++){
				sumAng += angPP[i];
				A += a[i];
			}

			deltaAng = 2 * PI - sumAng;
			curvature[c] = abs((3 * deltaAng) / A);

			radiusArr[c] = 1 / (curvature[c] + 1e-16);
			*/
			radiusArr[c] = std::min(1.0f/ heightDistance[c],1.5f);
			
			MString str = "c= ";
			str = str + c + " curvature[c]= " + curvature[c] + "\n";
			//MGlobal::displayInfo(MString("######OpenClothPBDNode Calculate curvature"+str));
		}
	}



}