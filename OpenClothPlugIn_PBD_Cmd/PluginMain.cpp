#include "OpenClothPBDNode.h"
#include "OPenClothPBDCmd.h"
#include <maya/MFnPlugin.h>


MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "Sim plug-in", "1.0", "Any");

	status = plugin.registerCommand("Sim", SimCmd::creator);
	if (!status) {
		status.perror("registerCommand failed");
		return status;
	}


	status = plugin.registerNode("SimNode", SimNode::id,
		SimNode::creator, SimNode::initialize);
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
	status = plugin.deregisterCommand("Sim");
	if (!status)
	{
		status.perror("deregisterCommand failed");
		return status;
	}
	status = plugin.deregisterNode(SimNode::id);
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}


