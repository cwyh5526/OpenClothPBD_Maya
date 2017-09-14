#include "OpenClothPBDNode.h"
#include "OPenClothPBDCmd.h"
#include <maya/MFnPlugin.h>


MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "OpenCloth plug-in", "1.0", "Any");

	status = plugin.registerCommand("OpenClothPBD", OpenClothPBDCmd::creator);
	if (!status) {
		status.perror("registerCommand failed");
		return status;
	}


	status = plugin.registerNode("OpenClothPBDNode", OpenClothPBDNode::id,
		OpenClothPBDNode::creator, OpenClothPBDNode::initialize);
	if (!status) {
		status.perror("registerNode failed");
		return status;
	}

	return status;
}

MStatus uninitializePlugin(MObject obj)
{
	MStatus	  status;
	MFnPlugin plugin(obj);
	status = plugin.deregisterCommand("OpenClothPBD");
	if (!status)
	{
		status.perror("deregisterCommand failed");
		return status;
	}
	status = plugin.deregisterNode(OpenClothPBDNode::id);
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}


