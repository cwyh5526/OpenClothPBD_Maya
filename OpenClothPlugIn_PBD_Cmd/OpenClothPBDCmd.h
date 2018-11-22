//
// Copyright (C) 2001 David Gould 
// 
#ifndef OPENCLOTHPBDCMD_H
#define OPENCLOTHPBDCMD_H

#include <maya/MPxCommand.h>
#include <maya/MDGModifier.h>

class SimCmd : public MPxCommand
{
public:
	virtual MStatus	doIt(const MArgList&);
	virtual MStatus undoIt();
	virtual MStatus redoIt();
	virtual bool isUndoable() const { return true; }

	static void *creator() { return new SimCmd; }
	static MSyntax newSyntax();

private:
	MDGModifier dgMod;
};

#endif