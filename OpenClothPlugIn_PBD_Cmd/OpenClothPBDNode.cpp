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
std::vector<glm::vec3> V; //velocity
std::vector<glm::vec3> F;
std::vector<float> W; //inverse particle mass 
std::vector<glm::vec3> Ri; //Ri = Xi-Xcm 

int oldX = 0, oldY = 0;
float rX = 15, rY = 0;
int state = 1;
float dist = -23;
const int GRID_SIZE = 10;

const size_t solver_iterations = 2; //number of solver iterations per step. PBD  

float kBend = 0.5f;//0.5f
float kStretch = 0.25f;//0.25f
float kDamp = 0.00125f;
glm::vec3 gravity = glm::vec3(0.0f, -0.00981f, 0.0f);
glm::vec3 nabi = glm::vec3(0.01f, 0.0f, 0.0f);//20170724

float mass = 1.f / 300;//50.f / (total_points);//1.f/totalpoints


double frameTimeQP = 0;
float frameTime = 0;


glm::vec3 Up = glm::vec3(0, 1, 0), Right, viewDir;
float startTime = 0;// fps = 0;
int totalFrames = 0;

glm::mat4 ellipsoid, inverse_ellipsoid;
int iStacks = 30;
int iSlices = 30;
float fRadius = 1;
// Resolve constraint in object space
glm::vec3 center = glm::vec3(0, 0, 0); //object space center of ellipsoid
float radius = 1;

MObject OpenClothPBDNode::time;
MObject OpenClothPBDNode::inputMesh;
MObject OpenClothPBDNode::outputMesh;
MTypeId OpenClothPBDNode::id(0x80001);

MObject OpenClothPBDNode::height; 	//2017.07.19
MObject OpenClothPBDNode::width;
MObject OpenClothPBDNode::subWidth;
MObject OpenClothPBDNode::subHeight;

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
	MGlobal::displayInfo("######OpenClothPBDNode initialize()");

	inputMesh = typedAttr.create("inputMesh", "in", MFnData::kMesh, &stat);
	if (!stat)
		return stat;

	outputMesh = typedAttr.create("outputMesh", "out", MFnData::kMesh, &stat);
	if (!stat)
		return stat;

	typedAttr.setStorable(false);

	//2017.07.19
	width = nAttr.create("width", "w", MFnNumericData::kDouble, 0.0);
	height = nAttr.create("height", "h", MFnNumericData::kDouble, 0.0);
	subWidth = nAttr.create("subWidth", "sw", MFnNumericData::kInt, 0);
	subHeight = nAttr.create("subHeight", "sh", MFnNumericData::kInt, 0);

	addAttribute(time);
	addAttribute(inputMesh);
	addAttribute(outputMesh);

	addAttribute(width);//2017.07.19
	addAttribute(height);//2017.07.19
	addAttribute(subWidth);//2017.07.19
	addAttribute(subHeight);//2017.07.19


	attributeAffects(inputMesh, outputMesh);
	attributeAffects(time, outputMesh);

	attributeAffects(width, outputMesh);//2017.07.19
	attributeAffects(height, outputMesh);//2017.07.19
	attributeAffects(subWidth, outputMesh);//2017.07.19
	attributeAffects(subHeight, outputMesh);//2017.07.19
	
	return MS::kSuccess;
}

MStatus OpenClothPBDNode::compute(const MPlug& plug, MDataBlock& data)

{
	MStatus returnStatus;
	//MGlobal::displayInfo("######OpenClothPBDNode compute()");
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
		size = wHnd.asDouble();
		hsize = hHnd.asDouble();

		MDataHandle subWHnd = data.inputValue(subWidth, &returnStatus);
		McheckErr(returnStatus, "Error getting subWidth data handle\n");
		MDataHandle subHHnd = data.inputValue(subHeight, &returnStatus);
		McheckErr(returnStatus, "Error getting subHeight data handle\n");

		//MGlobal::displayInfo("######OpenClothPBDNode compute() prevNumX"+prevNumX);

		prevNumX = numX;//20170724
		prevNumY = numY;//20170724

		numX = subWHnd.asInt();
		numY = subHHnd.asInt();
		total_points = (numX + 1)*(numY + 1);


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

	MFnMesh meshFn;
	meshFn.copy(inData, outData);
	meshFn.setObject(outData);

	returnStatus = meshFn.getPoints(iarr, MSpace::kObject);
	if (returnStatus != MS::kSuccess)
	{
		cerr << "Error in getting Vertices:" << returnStatus << endl;
		return MObject::kNullObj;
	}

	static bool first = true; //20170721
	if (time.value() == 1){
		first = true;
	}
	if (prevNumX != numX || prevNumY != numY) //20170817
	{
		first = true;
	}
	if (first){
		InitializeOpenCloth();
		first = false;
	}

	StepPhysics(timeStep);


	meshFn.setPoints(iarr);

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
	MGlobal::displayInfo("######OpenClothPBDNode InitializeOpenCloth()");

	size_t i = 0, j = 0, count = 0;
	int l1 = 0, l2 = 0;
	float ypos = 7.0f;
	int v = numY + 1;
	int u = numX + 1;

	//iarr.clear();//20170601 //20170901

	indices.resize(numX*numY * 2 * 3);

	X.resize(total_points);
	tmp_X.resize(total_points);
	V.resize(total_points);
	F.resize(total_points);
	Ri.resize(total_points);

	//fill in positions
	for (int i = 0; i < total_points; i++) {
		X[i].x = iarr[i].x;
		X[i].y = iarr[i].y;
		X[i].z = iarr[i].z;
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

	for (i = 0; i < numX-1; i++){//20170529 한줄 다 20170911 아래 위 다
		W[i] = 0.0;//맨아래줄
		W[total_points - i - 1] = 0.0;//맨 윗줄
		W[i*(numX + 1)] = 0.0; //왼쪽
		W[i*(numX + 1) - 1] = 0.0;//오른쪽
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
	/*
	//create a basic ellipsoid object
	ellipsoid = glm::translate(glm::mat4(1), glm::vec3(-1, -2, 0));
	ellipsoid = glm::rotate(ellipsoid, 45.0f, glm::vec3(1, 0, 0));
	ellipsoid = glm::scale(ellipsoid, glm::vec3(fRadius, fRadius, fRadius / 2));
	inverse_ellipsoid = glm::inverse(ellipsoid);
	*/

}
void OpenClothPBDNode::StepPhysics(float dt){

	ComputeForces();
	IntegrateExplicitWithDamping(dt);

	// for collision constraints
	UpdateInternalConstraints(dt);
	UpdateExternalConstraints();

	Integrate(dt);
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

	F[total_points / 2] += glm::vec3(100, 0, 0);//20170911 10의 힘을 z 방향으로 준다
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
	for (size_t i = 0; i<total_points; i++) {
		glm::vec4 X_0 = (inverse_ellipsoid*glm::vec4(tmp_X[i], 1));
		glm::vec3 delta0 = glm::vec3(X_0.x, X_0.y, X_0.z) - center;
		float distance = glm::length(delta0);
		if (distance < 1.0f) {
			delta0 = (radius - distance) * delta0 / distance;

			// Transform the delta back to original space
			glm::vec3 delta;
			glm::vec3 transformInv;
			transformInv = glm::vec3(ellipsoid[0].x, ellipsoid[1].x, ellipsoid[2].x);
			transformInv /= glm::dot(transformInv, transformInv);
			delta.x = glm::dot(delta0, transformInv);
			transformInv = glm::vec3(ellipsoid[0].y, ellipsoid[1].y, ellipsoid[2].y);
			transformInv /= glm::dot(transformInv, transformInv);
			delta.y = glm::dot(delta0, transformInv);
			transformInv = glm::vec3(ellipsoid[0].z, ellipsoid[1].z, ellipsoid[2].z);
			transformInv /= glm::dot(transformInv, transformInv);
			delta.z = glm::dot(delta0, transformInv);
			tmp_X[i] += delta;
			V[i] = glm::vec3(0);
		}
	}
}

void OpenClothPBDNode::UpdateExternalConstraints() {
	//EllipsoidCollision();
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
		if (tmp_X[total_points / 2].x > 3.0){
			tmp_X[total_points / 2].x = 3.0;
			W[total_points / 2] = 0.0;
		}
		//GroundCollision();
	}
}

