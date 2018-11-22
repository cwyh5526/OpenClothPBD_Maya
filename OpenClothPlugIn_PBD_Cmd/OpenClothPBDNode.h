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
#include <maya/MFnVectorArrayData.h>//20171205

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

/* 얘는 actuator들 만들고 위치, radius 연결하는 그것
int $j=0;
int $actnum=0;
string $groupStr="group -n Actuator";
for($j=1;$j<=420;$j=$j+1){

$actnum=$j-1;
polySphere -r 1 -sx 20 -sy 20 -ax 0 1 0 -cuv 2 -ch 1;
string $str ="float $radius=`getAttr OpenClothPBDNode1.ellipsoidRadius`; vector $actPos2[]=`getAttr OpenClothPBDNode1.actuatorPosition`;\nvector $position =$actPos2["+$actnum+"];\npSphere"+$j+".translateX=$position.x-$radius;\npSphere"+$j+".translateY=$position.y;\npSphere"+$j+".translateZ=$position.z;\n\n";
expression -string $str;
string $con = "OpenClothPBDNode1.ellipsoidRadius polySphere"+$j+".radius;";
eval("connectAttr "+ $con);
$groupStr=$groupStr+" pSphere"+$j;
}

eval($groupStr+";");
setAttr "lambert1.transparency" -type double3 0.6 0.6 0.6 ;

*/
/* 이것은 UI
string $windowName = "ChamelUI";
if (`window -exists $windowName`) deleteUI $windowName;

window  -title "Chameleon Surface Skin Simulator"
-resizeToFitChildren true $windowName;

columnLayout ;
text -label " " -align "right";//just for padding
string $sourceLayers[] = {"Wire Frame","Shading","Actuator Strain","Nodal Strain","Texture"};
rowColumnLayout -numberOfColumns 2 -columnWidth 1 143 -columnWidth 2 200;
text -label "Viewing Mode: " -align "right";
optionMenu -label "" -changeCommand "optionVar -sv \"cml_source\" \"#1\"" SourceLayerSelection;//"optionVar -sv \"view_mode\" \"#1\"";
for($s in $sourceLayers)
menuItem -l $s;
setParent ..;
attrFieldSliderGrp -label "Width: "
-attribute ("polyPlane1.width")
-fieldMinValue 35.34 -fieldMaxValue 100 -precision 2
-minValue 0 -maxValue 100 ;
attrFieldSliderGrp -label "Height: "
-attribute ("polyPlane1.height")
-fieldMinValue 18 -fieldMaxValue 100 -precision 2
-minValue 18 -maxValue 100;
attrFieldSliderGrp -label "Subdivisions Width: "
-attribute ("polyPlane1.subdivisionsWidth")
-fieldMinValue 56 -fieldMaxValue 300
-minValue 56 -maxValue 300 ;
attrFieldSliderGrp -label "Subdivisions Height: "
-attribute ("polyPlane1.subdivisionsHeight")
-fieldMinValue 15 -fieldMaxValue 300
-minValue 15 -maxValue 300 ;
attrFieldSliderGrp -label "Actuator Radius: "
-attribute ("OpenClothPBDNode1.ellipsoidRadius")
-fieldMinValue 0.01 -fieldMaxValue 10
-minValue 0.01 -maxValue 10 ;
rowColumnLayout -numberOfColumns 5 -columnWidth 1 143 -columnWidth 2 10 -columnWidth 3 50 -columnWidth 4 50;
text -label "Actuator Rendering: " -align "right";
text -label "  " -align "right";

radioCollection actRender;
radioButton -label "On" -onCommand actuatorOnCmd ;
radioButton -label "Off" -select -onCommand actuatorOffCmd ;
radioButton -label "Delete" -select -onCommand deleteActuatorCmd ;
setParent ..;

string $filepath = `getAttr "OpenClothPBDNode1.heightFilePath"`;
rowColumnLayout -numberOfColumns 5 -columnWidth 1 143 -columnWidth 2 150 -columnWidth 4 75;
text -label "Height File Path: " -align "right";
textField -w 150 -tx $filepath heightfilepath;
button -label "Browse" -align "center" -width 75 -height 10 -command browseBtnCmd;
setParent ..;

text -label " " -align "right";//just for padding
gridLayout -cellWidthHeight 130 25 -numberOfRowsColumns 1 3;
text -label " ";
button -label "Change" -align "center" -width 133 -height 10  -command buttonCmd;
text -label " ";
setParent ..;

text -label " " -align "right";//just for padding

showWindow;

global proc buttonCmd() {
string $matteLayer = `optionMenu -q -v SourceLayerSelection`;
string $window = `getPanel -withFocus`;//example modelPanel1
modelEditor -e -wos 0 $window;
switch($matteLayer){
case "Wire Frame" :
if( `modelEditor -q -wos $window` == 0 ){
modelEditor -e -wos 1 $window;
}
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

modelEditor  -e -tx true $window;


break;
}
}

global proc actuatorOnCmd() {
if(!(`objExists Actuator`)){
int $j=0;
int $actnum=0;
string $groupStr="group -n Actuator";
for($j=1;$j<=420;$j=$j+1){
$actnum=$j-1;
polySphere -r 1 -sx 20 -sy 20 -ax 0 1 0 -cuv 2 -ch 1;
string $str ="float $radius=`getAttr OpenClothPBDNode1.ellipsoidRadius`; vector $actPos2[]=`getAttr OpenClothPBDNode1.actuatorPosition`;\nvector $position =$actPos2["+$actnum+"];\npSphere"+$j+".translateX=$position.x-$radius;\npSphere"+$j+".translateY=$position.y;\npSphere"+$j+".translateZ=$position.z;\n\n";
expression -string $str;
string $con = "OpenClothPBDNode1.ellipsoidRadius polySphere"+$j+".radius;";
eval("connectAttr "+ $con);
$groupStr=$groupStr+" pSphere"+$j;
}
eval($groupStr+";");
select -cl;
}
select -r Actuator;
ShowSelectedObjects;
select -cl;
setAttr "lambert1.transparency" -type double3 0.3 0.3 0.3 ;
}
global proc actuatorOffCmd() {
select -r Actuator;
HideSelectedObjects;
setAttr "lambert1.transparency" -type double3 0.0 0.0 0.0 ;
}

global proc deleteActuatorCmd() {
delete Actuator;

}

global proc browseBtnCmd(){
fileBrowser("onFileOpen","Open","",0);
}
global proc int onFileOpen(string $filepath, string $type){
setAttr "OpenClothPBDNode1.heightFilePath" -type "string" $filepath;
textField -edit -tx $filepath heightfilepath;
return true;
}

*/
class SimNode : public MPxNode
{
public:
	SimNode() {};
	virtual 		~SimNode() {};
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
	static MObject actuatorPosition;//20171205
	static MObject heightFilePath;//20180122

	//material model property
	float avg_Strain;
	float k;
	std::vector<bool> positive_move;

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