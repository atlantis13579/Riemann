#pragma once

#include "sap.h"
#include "GeometryObject.h"

// http://www.codercorner.com/SAP.pdf
void sap_incremental(std::vector<sweep_point> axis[3], std::vector<GeometryObject*>& objs, std::set<sap_key>* overlaps, bool dirty, int axis_filter = 7);
