#ifndef OPENCLOTHPBDNODE_H
#define OPENCLOTHPBDNODE_H

#include <vector>
#include <algorithm>
#include <math.h>

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
<시나리오>
setting 변경
mel 실행
재생
버튼 막 누르기 ㅎㅎ
*/
/*
{
string $windowName = "ChamelUI";
if (`window -exists $windowName`) deleteUI $windowName;

window -title "Chameleon Surface Cloth Simulator" $windowName;
columnLayout ;
string $sourceLayers[] = {"Wire Frame","Shading","Actuator Strain","Nodal Strain","Texture"};
string $matteLayerSelectionOpts = `rowColumnLayout -numberOfColumns 2 -columnWidth 1 142 -columnWidth 2 150`;
text -l "                     Viewing Mode";
optionMenu -l "" -cc "optionVar -sv \"cml_source\" \"#1\"" SourceLayerSelection;
for($s in $sourceLayers)
menuItem -l $s;
//menuItem -label "Wire Frame" -dragMenuCommand "";;
// menuItem -label "Shading" -dragMenuCommand "setAttr OpenClothPBDNode1.colorDimension 0;";
// menuItem -label "Actuator Strain"-dragMenuCommand "setAttr OpenClothPBDNode1.colorDimension 1;";
// menuItem -label "Nodal Strain" -dragMenuCommand "setAttr OpenClothPBDNode1.colorDimension 2;";
// menuItem -label "Texture"-dragMenuCommand "setAttr OpenClothPBDNode1.colorDimension 2;";
setParent ..;
string $sourceLayerDefault = (`optionVar -ex "cml_source"`) ? `optionVar -q "cml_source"` : "masterLayer";

floatSliderGrp -label "Width" -field true
-fieldMinValue 0 -fieldMaxValue 100 -precision 2
-minValue 0 -maxValue 100 -value 35.34;
floatSliderGrp -label "Height" -field true
-fieldMinValue 0 -fieldMaxValue 100 -precision 2
-minValue 0 -maxValue 100 -value 18.00;
intSliderGrp -label "Subdivisions Width" -field true
-fieldMinValue 0 -fieldMaxValue 300
-minValue 0 -maxValue 300 -value 224;
intSliderGrp -label "Subdivisions Height" -field true
-fieldMinValue 0 -fieldMaxValue 300
-minValue 0 -maxValue 300 -value 120;
button -label "Change" -c buttonCmd;
showWindow;
}

global proc buttonCmd()
{
string $matteLayer = `optionMenu -q -v SourceLayerSelection`;
switch($matteLayer){
case "Wire Frame" :
break;
case "Shading" :
setAttr OpenClothPBDNode1.colorDimension 0;
break;
case "Actuator Strain" :
setAttr OpenClothPBDNode1.colorDimension 1;
break;
case "Nodal Strain" :
setAttr OpenClothPBDNode1.colorDimension 2;
break;
case "Texture" :
break;
}
};
*/
/*
string $pPlane1[] = `polyPlane -w 35.34 -h 18 -sx 224 -sy 120 -ax 1 0 0 -cuv 2 -ch 1`;
string $pPlane2[] = `polyPlane -w 35.34 -h 18 -sx 224 -sy 120 -ax 1 0 0 -cuv 2 -ch 1`;
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

select -r pPlane2;
setAttr OpenClothPBDNode1.colorDimension 1;
toggleShadeMode();
select -cl;
*/
/*
if (`window -exists myWindow`) deleteUI myWindow;
window -title "new window" -widthHeight 300 200 myWindow;
if (`windowPref -exists myWindow`) windowPref -remove myWindow;
// columnLayout;
// columnLayout -adjustableColumn true;
columnLayout -columnAttach "both" 5 -rowSpacing 5 -columnWidth 100;
button -label "shading" -command "setAttr OpenClothPBDNode1.colorDimension 0;";
button -label "Actuator Strain" -command "setAttr OpenClothPBDNode1.colorDimension 1;";
button -label "Nodal Strain" -command "setAttr OpenClothPBDNode1.colorDimension 2;";
showWindow myWindow;
*/
/*
proc no_texture(){
setAttr "OpenClothPBDNode1.colorDimension" 0;
}
proc actuator(){
setAttr "OpenClothPBDNode1.colorDimension" 1;
}
proc nodal(){
setAttr "OpenClothPBDNode1.colorDimension" 2;
}

window;
columnLayout;
button -label "no texture" -command "no_texture";
button -label "actuator" -command "actuator";
button -label "nodal"  -command "nodal";
showWindow;
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

	//strain configuration
	static MObject colorDim;

	//radius configuration
	static MObject ellipRadius; //20171127

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
	void UpdatePositionConstraint();//20170925

	void GroundCollision();
	void EllipsoidCollision();
	void EllipsoidMove();

	void UpdateExternalConstraints();
	void Integrate(float deltaTime);

	void CalActuatorStrain();
	void CalNodalStrain();

	void CalEllipsoidRadius();

	void InitializeOpenCloth();
private:
	static MFloatPointArray iarr;

};

#endif