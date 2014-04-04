#ifndef _OSMAND_GENERAL_ROUTER_CPP
#define _OSMAND_GENERAL_ROUTER_CPP
#include "generalRouter.h"
#include "binaryRoutePlanner.h"
#include <sstream>
const int RouteAttributeExpression::LESS_EXPRESSION = 1;
const int RouteAttributeExpression::GREAT_EXPRESSION = 1;


float parseFloat(MAP_STR_STR & attributes, std::string const & key, float def) {
	if(attributes.find(key) != attributes.end() && attributes[key] != "") {
		return atof(attributes[key].c_str());
	}
	return def;
}

bool parseBool(MAP_STR_STR & attributes, std::string const & key, bool def) {
	if (attributes.find(key) != attributes.end() && attributes[key] != "") {
		return attributes[key] == "true";
	}
	return def;
}

std::string const & parseString(MAP_STR_STR & attributes, std::string const & key, std::string const & def) {
	if (attributes.find(key) != attributes.end() && attributes[key] != "") {
		return attributes[key];
	}
	return def;
}

dynbitset& increaseSize(dynbitset& t, uint targetSize) {
	if(t.size() < targetSize) {
		t.resize(targetSize);
	}
	return t;
}

dynbitset& align(dynbitset& t, uint targetSize) {
	if(t.size() < targetSize) {
		t.resize(targetSize);
	} else if(t.size() > targetSize) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Bitset %d is longer than target %d", t.size(), targetSize);
	}
	return t;
}


double parseValue(std::string const & value, std::string const & type) {
	double vl = -1;
	if("speed" == type) {
		vl = RouteDataObject::parseSpeed(value, vl);
	} else if("weight" == type) {
		vl = RouteDataObject::parseWeightInTon(value, vl);
	} else if("length" == type) {
		vl = RouteDataObject::parseLength(value, vl);
	} else {
		int i = findFirstNumberEndIndex(value);
		if (i > 0) {
			// could be negative
			return atof(value.substr(0, i).c_str());
		}
	}
	if(vl == -1) {
		return DOUBLE_MISSING;
	}
	return vl;
}


void GeneralRouter::addAttribute(std::string const & k, std::string const & v) {
	attributes[k] = v;
	if(k=="restrictionsAware") {
		_restrictionsAware = parseBool(attributes, v, _restrictionsAware);
	} else if(k=="leftTurn") {
		leftTurn = parseFloat(attributes, v, leftTurn);
	} else if(k=="rightTurn") {
		rightTurn = parseFloat(attributes, v, rightTurn);
	} else if(k=="roundaboutTurn") {
		roundaboutTurn = parseFloat(attributes, v, roundaboutTurn);
	} else if(k=="minDefaultSpeed") {
		minDefaultSpeed = parseFloat(attributes, v, minDefaultSpeed * 3.6f) / 3.6f;
	} else if(k =="maxDefaultSpeed") {
		maxDefaultSpeed = parseFloat(attributes, v, maxDefaultSpeed * 3.6f) / 3.6f;
	}
}

void toStr(std::ostringstream& s, UNORDERED(set)<std::string> const & v) {
	s << "[";
	UNORDERED(set)<std::string>::const_iterator i = v.begin();
	for(; i != v.end(); i++) {
		if(i != v.begin()) s <<", ";
		s << *i;
	}

	s << "]";
}

void RouteAttributeEvalRule::printRule(GeneralRouter const * r) const {
	std::ostringstream s;
	s << " Select ";
	if(selectValue == DOUBLE_MISSING) {
		s << selectValueDef;
	} else {
		s << selectValue;
	}

	bool f = true;
	for(uint k = 0; k < filterTypes.size(); k++) {
		if(filterTypes.test(k)) {
			if(f) {
				s << " if ";
				f = !f;
			}
			tag_value key = r->universalRulesById[k];
			s << key.first << "/" << key.second;
		}
	}
	f = true;
	for(uint k = 0; k < filterNotTypes.size(); k++) {		
		if(filterNotTypes.test(k)) {
			if(f) {
				s << " if ";
				f = !f;
			}
			tag_value key = r->universalRulesById[k];
			s << key.first << "/" << key.second;
		}
	}
	for(uint k = 0; k < parameters.size(); k++) {
		s << " param=" << parameters[k];
	}
	if(onlyTags.size() > 0) {
		s << " match tag = ";
		toStr(s, onlyTags);
	}
	if(onlyNotTags.size() > 0) {
		s << " not match tag = ";
		toStr(s, onlyNotTags);
	}
	if(expressions.size() > 0) {
		s << " subexpressions " << expressions.size();
	}
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%s", s.str().c_str());
}

RouteAttributeExpression::RouteAttributeExpression(std::vector<std::string> const & vls, int type, std::string const & vType) :
		 values(vls), expressionType(type), valueType(vType){
	cacheValues.resize(vls.size());
	for (uint i = 0; i < vls.size(); i++) {
		if(vls[i][0] != '$' && vls[i][0] != ':') {
			double o = parseValue(vls[i], valueType);
			if (o != DOUBLE_MISSING) {
				cacheValues[i] = o;
			}
		}
	}
}

void RouteAttributeEvalRule::registerAndTagValueCondition(GeneralRouter* r, std::string const & tag, std::string const & value, bool nt) {
	tagValueCondDefTag.push_back(tag);
	tagValueCondDefValue.push_back(value);
	tagValueCondDefNot.push_back(nt);
	if(value == "") { 
		if (nt) {
			onlyNotTags.insert(tag);
		} else {
			onlyTags.insert(tag);
		}
	} else {
		tag_value t = tag_value(tag, value);
		int vtype = r->registerTagValueAttribute(t);
		if(nt) {
			increaseSize(filterNotTypes, vtype + 1).set(vtype);
		} else {
			increaseSize(filterTypes, vtype + 1).set(vtype);
		}
	}
}

void RouteAttributeEvalRule::registerParamConditions(std::vector<std::string> const & params) {
	parameters.insert(parameters.end(), params.begin(), params.end());
}

void RouteAttributeEvalRule::registerSelectValue(std::string const & value, std::string const & type) {
	this->selectType = type;
	this->selectValueDef = value;
	if (selectValueDef.length() > 0 && (selectValueDef[0] == '$' || selectValueDef[0] == ':')) {
		// init later
		selectValue = DOUBLE_MISSING;
	} else {
		selectValue = parseValue(value, type);
		if(selectValue == DOUBLE_MISSING) {
		//	System.err.println("Routing.xml select value '" + value+"' was not registered");
		}
	}
}


double GeneralRouter::parseValueFromTag(uint id, std::string const & type, GeneralRouter const * router) {
	while (ruleToValue.size() <= id) {
		ruleToValue.push_back(DOUBLE_MISSING);
	}
	double res = ruleToValue[id];
	if (res == DOUBLE_MISSING) {
		tag_value const & v = router->universalRulesById[id];
		res = parseValue(v.second, type);
		if (res == DOUBLE_MISSING) {
			res = DOUBLE_MISSING - 1;
		}
		ruleToValue[id] = res;
	}
	if (res == DOUBLE_MISSING - 1) {
		return DOUBLE_MISSING;
	}
	return res;
}


bool GeneralRouter::containsAttribute(std::string const & attribute) const {
	return attributes.find(attribute) != attributes.end();
}

std::string const & GeneralRouter::getAttribute(std::string const & attribute) {
	return attributes[attribute];
}

bool GeneralRouter::acceptLine(SHARED_PTR<RouteDataObject> const & way) {
	int res = getObjContext(RouteDataObjectAttribute::ACCESS).evaluateInt(way, 0);
///OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "accept %lld %d", way->id, res);///
	return res >= 0;
}

int GeneralRouter::isOneWay(SHARED_PTR<RouteDataObject> const & road) {
	return getObjContext(RouteDataObjectAttribute::ONEWAY).evaluateInt(road, 0);
}

double GeneralRouter::defineObstacle(SHARED_PTR<RouteDataObject> const & road, uint point) {
	if(road->pointTypes.size() > point && road->pointTypes[point].size() > 0){
		return getObjContext(RouteDataObjectAttribute::OBSTACLES).evaluateDouble(road->region, road->pointTypes[point], 0);
	}
	return 0;
}

double GeneralRouter::defineRoutingObstacle(SHARED_PTR<RouteDataObject> const & road, uint point) {
	if(road->pointTypes.size() > point && road->pointTypes[point].size() > 0){
		return getObjContext(RouteDataObjectAttribute::ROUTING_OBSTACLES).evaluateDouble(road->region, road->pointTypes[point], 0);
	}
	return 0;
}

double GeneralRouter::defineRoutingSpeed(SHARED_PTR<RouteDataObject> const & road) {
	return std::min(defineVehicleSpeed(road), maxDefaultSpeed);
}

double GeneralRouter::defineVehicleSpeed(SHARED_PTR<RouteDataObject> const & road) {
	return getObjContext(RouteDataObjectAttribute::ROAD_SPEED) .evaluateDouble(road, getMinDefaultSpeed() * 3.6) / 3.6;
}

double GeneralRouter::defineSpeedPriority(SHARED_PTR<RouteDataObject> const & road) {
	return getObjContext(RouteDataObjectAttribute::ROAD_PRIORITIES).evaluateDouble(road, 1.);
}

double GeneralRouter::getMinDefaultSpeed() const {
	return minDefaultSpeed;
}

double GeneralRouter::getMaxDefaultSpeed() const {
	return maxDefaultSpeed;
}

bool GeneralRouter::restrictionsAware() const {
	return _restrictionsAware;
}

double GeneralRouter::calculateTurnTime(SHARED_PTR<RouteSegment> const & segment, int segmentEnd,
		SHARED_PTR<RouteSegment> const & prev, int prevSegmentEnd) const {
	if(prev->road->pointTypes.size() > (uint)prevSegmentEnd && prev->road->pointTypes[prevSegmentEnd].size() > 0){
		RoutingIndex* reg = prev->getRoad()->region;
		std::vector<uint32_t> pt = prev->road->pointTypes[prevSegmentEnd];
		for (uint i = 0; i < pt.size(); i++) {
			tag_value r = reg->decodingRules[pt[i]];
			if ("highway" == r.first && "traffic_signals" == r.second) {
				// traffic signals don't add turn info
				return 0;
			}
		}
	}
	
	if(segment->getRoad()->roundabout() && !prev->getRoad()->roundabout()) {
		double rt = roundaboutTurn;
		if(rt > 0) {
			return rt;
		}
	}
	if (leftTurn > 0 || rightTurn > 0) {
		double a1 = segment->getRoad()->directionRoute(segment->getSegmentStart(), segment->getSegmentStart() < segmentEnd);
		double a2 = prev->getRoad()->directionRoute(prevSegmentEnd, prevSegmentEnd < prev->getSegmentStart());
		double diff = abs(alignAngleDifference(a1 - a2 - M_PI));
		// more like UT
		if (diff > 2 * M_PI / 3) {
			return leftTurn;
		} else if (diff > M_PI / 2) {
			return rightTurn;
		}
		return 0;
	}
	return 0;
}

uint GeneralRouter::registerTagValueAttribute(const tag_value& r) {
	std::string key = r.first + "$" + r.second;
	MAP_STR_INT::iterator it = universalRules.find(key);
	if(it != universalRules.end()) {
		return ((uint)it->second);
	}
	uint id = universalRules.size();
	universalRulesById.push_back(r);
	universalRules[key] = id;
	dynbitset& d = increaseSize(tagRuleMask[r.first], id + 1);
	d.set(id);
	return id;
}

dynbitset RouteAttributeContext::convert(RoutingIndex* reg, std::vector<uint32_t>& types) const {
	dynbitset b(router->universalRules.size());
	///MAP_INT_INT map = router->regionConvert[reg];
	for(uint k = 0; k < types.size(); k++) {
		///MAP_INT_INT::const_iterator nid = map.find(types[k]);
		int vl;
		///if(nid == map.end()){
			tag_value const & r = reg->decodingRules[types[k]];
			vl = router->registerTagValueAttribute(r);
			///map[types[k]] = vl;
		///} else {
			///vl = nid->second;
		///}
		increaseSize(b, router->universalRules.size()).set(vl);
	}
	return b;
}

double RouteAttributeEvalRule::eval(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter* router) {
	if (matches(types, paramContext, router)) {
		return calcSelectValue(types, paramContext, router);
	}
	return DOUBLE_MISSING;
}


double RouteAttributeEvalRule::calcSelectValue(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter* router) {
	if(selectValue != DOUBLE_MISSING) {
		return selectValue;
	}
	if (selectValueDef.length() > 0 && selectValueDef[0]=='$') {
		UNORDERED(map)<std::string, dynbitset >::iterator ms = router->tagRuleMask.find(selectValueDef.substr(1));
		if (ms != router->tagRuleMask.end() && align(ms->second, types.size()).intersects(types)) {
			dynbitset findBit(ms->second.size());
			findBit |= ms->second;
			findBit &= types;
			uint value = findBit.find_first();
			double vd = router->parseValueFromTag(value, selectType, router);
			return vd;
		}
	} else if (selectValueDef.length() > 0 && selectValueDef[0]==':') {
		std::string p = selectValueDef.substr(1);
		MAP_STR_STR::const_iterator it = paramContext.vars.find(p);
		if (it != paramContext.vars.end()) {
			selectValue = parseValue(it->second, selectType);
		} else {
			return DOUBLE_MISSING;
		}
	}
	return selectValue;
}

bool RouteAttributeExpression::matches(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter* router) const {
	double f1 = calculateExprValue(0, types, paramContext, router);
	double f2 = calculateExprValue(1, types, paramContext, router);
	if(f1 == DOUBLE_MISSING || f2 == DOUBLE_MISSING) {
		return false;
	}
	if (expressionType == LESS_EXPRESSION) {
		return f1 <= f2;
	} else if (expressionType == GREAT_EXPRESSION) {
		return f1 >= f2;
	}
	return false;
}


double RouteAttributeExpression::calculateExprValue(int id, dynbitset const & types, ParameterContext const & paramContext, GeneralRouter* router) const {
	std::string const & value = values[id];
	// TODO initialize cacheValues correctly
	double cacheValue = cacheValues[id];
	if(cacheValue != DOUBLE_MISSING) {
		return cacheValue;
	}
	double o = DOUBLE_MISSING;

	if (value.length() > 0 && value[0]=='$') {
		UNORDERED(map)<std::string, dynbitset >::iterator ms = router->tagRuleMask.find(value.substr(1));
		if (ms != router->tagRuleMask.end() && align(ms->second, types.size()).intersects(types)) {
			dynbitset findBit(ms->second.size());
			findBit |= ms->second;
			findBit &= types;
			uint value = findBit.find_first();
			return router->parseValueFromTag(value, valueType, router);
		}
	} else if (value.length() > 0 && value[0]==':') {
		std::string p = value.substr(1);
		MAP_STR_STR::const_iterator it = paramContext.vars.find(p);
		if (it != paramContext.vars.end()) {
			o = parseValue(it->second, valueType);
		} else {
			return DOUBLE_MISSING;		
		}
	}
	return o;
} 

bool RouteAttributeEvalRule::matches(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter * router) {
	if(!checkAllTypesShouldBePresent(types)) {
		return false;
	}
	if(!checkAllTypesShouldNotBePresent(types)) {
		return false;
	}
	if(!checkFreeTags(types, router)) {
		return false;
	}
	if(!checkNotFreeTags(types, router)) {
		return false;
	}
	if(!checkExpressions(types, paramContext, router)) {
		return false;
	}
	return true;
}

bool RouteAttributeEvalRule::checkExpressions(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter * router) const {
	for(uint i = 0; i < expressions.size(); i++) {
		if(!expressions[i].matches(types, paramContext, router)) {
			return false;
		}
	}
	return true;
}

bool RouteAttributeEvalRule::checkFreeTags(dynbitset const & types, GeneralRouter * router) const {
	for(UNORDERED(set)<std::string>::const_iterator it = onlyTags.begin(); it != onlyTags.end(); it++) {
		UNORDERED(map)<std::string, dynbitset >::iterator ms = router->tagRuleMask.find(*it);
		if (ms == router->tagRuleMask.end() || !align(ms->second, types.size()).intersects(types)) {
			return false;
		}
	}
	return true;
}

bool RouteAttributeEvalRule::checkNotFreeTags(dynbitset const & types, GeneralRouter * router) const {
	for(UNORDERED(set)<std::string>::const_iterator it = onlyNotTags.begin(); it != onlyNotTags.end(); it++) {
		UNORDERED(map)<std::string, dynbitset>::iterator ms = router->tagRuleMask.find(*it);
		if (ms != router->tagRuleMask.end() && align(ms->second, types.size()).intersects(types)) {
			return false;
		}
	}
	return true;
}

bool RouteAttributeEvalRule::checkAllTypesShouldNotBePresent(dynbitset const & types) {
	return ! align(filterNotTypes, types.size()).intersects(types);
}

bool RouteAttributeEvalRule::checkAllTypesShouldBePresent(dynbitset const & types) {
	return align(filterTypes, types.size()).is_subset_of(types);
}

#endif /*_OSMAND_GENERAL_ROUTER_CPP*/
