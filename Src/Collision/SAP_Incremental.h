#pragma once

#include "sap.h"
#include "GeometryObject.h"

void sap_incremental_init(std::vector<GeometryObject*>& objs, int k, std::vector<sweep_point>& axis);

// http://www.codercorner.com/SAP.pdf
void sap_incremental(std::vector<std::vector<sweep_point>>& axis, std::vector<GeometryObject*>& objs, std::set<sap_key>* overlaps);
