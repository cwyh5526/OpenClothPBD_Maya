#ifndef OPENCLOTHPBDNODE_H
#define OPENCLOTHPBDNODE_H

#include <vector>
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //for matrices
#include <glm/gtc/type_ptr.hpp>


#include <maya/MTime.h>
#include <maya/MFnMesh.h>
#include <maya/MPoint.h>
#include <maya/MFloatPoint.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFloatArray.h>
#include <maya/MIntArray.h>
#include <maya/MDoubleArray.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>

#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnMesh.h>

#include <maya/MFnMeshData.h>

#include <maya/MIOStream.h>
#include <maya/MFnNObjectData.h>
#include <maya/MGlobal.h>
#include <maya/MAnimControl.h>

#include <maya/MColor.h>
#include <maya/MItMeshPolygon.h>

/*
string $pPlane1[] = `polyPlane -w 10 -h 10 -sx 200 -sy 200 -ax 1 0 0 -cuv 2 -ch 1`;
string $pPlane2[] = `polyPlane -w 10 -h 10 -sx 200 -sy 200 -ax 1 0 0 -cuv 2 -ch 1`;
select -r polyPlane2 ;
doDelete;

createNode OpenClothPBDNode;
connectAttr polyPlane1.height OpenClothPBDNode1.height;
connectAttr polyPlane1.width OpenClothPBDNode1.width;
connectAttr polyPlane1.subdivisionsHeight OpenClothPBDNode1.subHeight;
connectAttr polyPlane1.subdivisionsWidth OpenClothPBDNode1.subWidth;

connectAttr time1.outTime OpenClothPBDNode1.time;
connectAttr pPlaneShape1.worldMesh[0] OpenClothPBDNode1.inputMesh;
//delete pPlane1;

connectAttr OpenClothPBDNode1.outputMesh pPlaneShape2.inMesh;
select -r pPlane1;
toggleVisibilityAndKeepSelection `optionVar -query toggleVisibilityAndKeepSelectionBehaviour`;
*/

class OpenClothPBDNode : public MPxNode
{
public:
	OpenClothPBDNode() {};
	virtual 		~OpenClothPBDNode() {};
	virtual MStatus compute(const MPlug& plug, MDataBlock& data); //brain of a node
	static  void*	creator();		//allows Maya to instantialte instances of this node
	static  MStatus initialize();	//define the I/O of node. called once immediately after a plug-in is loaded

	/*node attributes*/
	static MObject	time;
	static MObject  inputMesh;
	static MObject	outputMesh;


	static MObject height; 	//2017.07.19
	static MObject width;
	static MObject subHeight;
	static MObject subWidth;

	/*node type*/
	static MTypeId	id; //unique identifier used by create() to identify which node to create

protected:
	MObject createCloth(const MTime& time, MObject& inData, MObject& outData, MStatus& stat);

	void StepPhysics(float dt);
	void ComputeForces();

	static void IntegrateExplicitWithDamping(float deltaTime);

	void UpdateInternalConstraints(float deltaTime);
	void UpdateDistanceConstraint(int i);
	void UpdateBendingConstraint(int index);
	void UpdatePositionConstraint(float limit);//20170925

	void GroundCollision();
	void EllipsoidCollision();
	void EllipsoidMove(float limit);

	void UpdateExternalConstraints();
	void Integrate(float deltaTime);
	
	void CalStrain();

	void InitializeOpenCloth();
private:
	static MFloatPointArray iarr;

};

#endif