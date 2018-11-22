#include "OpenClothPBDNode.h"
#include "OpenClothPBDCmd.h"

#include <maya/MSelectionList.h>
#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MFnTransform.h>
#include <maya/MItSelectionList.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MTime.h>
#include <maya/MAnimControl.h>
#include <assert.h>

MStatus SimCmd::doIt(const MArgList &args)
{
	//MGlobal::displayInfo("######OpenClothPBDCmd doIt()");
	MStatus stat;

	// Get a list of currently selected objects
	MSelectionList selection;
	MGlobal::getActiveSelectionList(selection);

	MTime startTime = MAnimControl::minTime();
	MTime endTime = MAnimControl::maxTime();

	// Iterate over all the Mesh nodes
	MItSelectionList iter(selection, MFn::kMesh);
	for (; !iter.isDone(); iter.next())
	{
		// Get the shape node	
		MObject shapeNode;
		iter.getDependNode(shapeNode);
		MFnDependencyNode shapeFn(shapeNode);

		MPlug inMeshPlug = shapeFn.findPlug("inMesh");
		MPlug worldMeshPlugs = shapeFn.findPlug("worldMesh");
		MPlug worldMeshPlug = worldMeshPlugs.child(0);

		// Determine the input connections to the create plug
		MPlugArray srcPlugs;
		inMeshPlug.connectedTo(srcPlugs, true, false);
		assert(srcPlugs.length() == 1);
		MPlug srcPlug = srcPlugs[0];// plug�� �̸��� �ƴϱ� �̷��� �� �� �ƴ϶� findPlug�� ����
		MFnDependencyNode polyPlaneFn = srcPlug.node(); //get the polyPlane Node.

		//polyPlane1 plugs
		MPlug srcHeightPlug = polyPlaneFn.findPlug("height");
		MPlug srcWidthPlug = polyPlaneFn.findPlug("width");
		MPlug srcSubHeightPlug = polyPlaneFn.findPlug("subdivisionsHeight");
		MPlug srcSubWidthPlug = polyPlaneFn.findPlug("subdivisionsWidth");

		// Create new Sim node
		MObject SimNode = dgMod.createNode(SimNode::id);
		assert(!SimNode.isNull());
		MFnDependencyNode SimFn(SimNode);

	
		//MPlug timePlug = OpenClothPBDFn.findPlug("time");

		MPlug outputMeshPlug = SimFn.findPlug("outputMesh");
		MPlug inputMeshPlug = SimFn.findPlug("inputMesh");

		MPlug heightPlug = SimFn.findPlug("height");
		MPlug widthPlug = SimFn.findPlug("width");
		MPlug subHeightPlug = SimFn.findPlug("subHeight");
		MPlug subWidthPlug = SimFn.findPlug("subWidth");

		// Make the connections			
		dgMod.connect(srcHeightPlug, heightPlug);
		dgMod.connect(srcWidthPlug, widthPlug);
		dgMod.connect(srcSubHeightPlug, subHeightPlug);
		dgMod.connect(srcSubWidthPlug, subWidthPlug);

		//dgMod.connect(worldMeshPlug, inputMeshPlug);
		dgMod.disconnect(srcPlug, inMeshPlug);
		dgMod.connect(outputMeshPlug, inMeshPlug);


	

		static int i = 1;
		
		MString name = MString("simNode") + i++;
		dgMod.renameNode(SimNode, name);

		MString cmd;
		cmd = MString("connectAttr time1.outTime " + name + ".time");
		dgMod.commandToExecute(cmd);
		MGlobal::displayInfo(cmd);

		cmd = MString("connectAttr " + shapeFn.name() + ".worldMesh[0] " + name + ".inputMesh");
		dgMod.commandToExecute(cmd);
		//MGlobal::displayInfo(cmd);

		cmd = MString("disconnectAttr " + shapeFn.name() + ".worldMesh[0] " + name + ".inputMesh");
		dgMod.commandToExecute(cmd);
		MGlobal::displayInfo(cmd);
		//20170829
		/*
		cmd = MString("select -r" + shapename);
		dgMod.commandToExecute(cmd);
		MGlobal::displayInfo(cmd);

		cmd = MString("doDelete");
		dgMod.commandToExecute(cmd);
		MGlobal::displayInfo(cmd);
		*/



		//MString n( meltFn.name() );
		//MGlobal::displayInfo( "\nMelt node: " + name + " " + shapeFn.name() );
		/*
		MString cmd;
		cmd = MString("setKeyframe -at amount -t ") + startTime.value() + " -v " + 0.0 + " " + name;
		dgMod.commandToExecute(cmd);

		//MGlobal::displayInfo( cmd );

		cmd = MString("setKeyframe -at amount -t ") + endTime.value() + " -v " + 1.0 + " " + name;
		dgMod.commandToExecute(cmd);

		//MGlobal::displayInfo( cmd );
		*/
		/*
		//MPlug interestPlug = meltFn.findPlug( "isHistoricallyInteresting
		// Setup animation
		MPlug amountPlug = meltFn.findPlug( "amount" );

		MAnimControl::setCurrentTime( startTime );
		amountPlug.setValue( double(0.0) );
		MAnimControl::setCurrentTime( endTime );
		amountPlug.setValue( double(1.0) );
		*/
	}

	/*
	MAnimControl::setCurrentTime( curTime );
	MAnimControl::setAutoKeyMode( autoKeyOn );
	*/

	return redoIt();
}

MStatus SimCmd::undoIt()
{
	return dgMod.undoIt();
}

MStatus SimCmd::redoIt()
{
	return dgMod.doIt();
}

