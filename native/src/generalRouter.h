#ifndef _OSMAND_GENERAL_ROUTER_H
#define _OSMAND_GENERAL_ROUTER_H

#include "Common.h"
#include "common2.h"
#include <algorithm>
#include "boost/dynamic_bitset.hpp"
#include "Logging.h"
//#include "binaryRead.h"
#include "RoutingIndex.hpp"

struct RouteSegment;
class GeneralRouter;
class RouteAttributeContext;

typedef UNORDERED(map)<std::string, float> MAP_STR_FLOAT;
typedef UNORDERED(map)<std::string, std::string> MAP_STR_STR;
typedef UNORDERED(map)<int, int> MAP_INT_INT;
typedef UNORDERED(map)<std::string, int> MAP_STR_INT;
typedef boost::dynamic_bitset<> dynbitset;

#define DOUBLE_MISSING -1.1e9 // random big negative number


enum class RouteDataObjectAttribute : unsigned int {
	ROAD_SPEED = 0, //"speed"
	ROAD_PRIORITIES = 1, // "priority"
	ACCESS = 2, // "access"
	OBSTACLES = 3, // "obstacle_time"
	ROUTING_OBSTACLES = 4, // "obstacle"
	ONEWAY = 5,// "oneway"
	PENALTY_TRANSITION = 6 // 
};

enum class GeneralRouterProfile {
	CAR,
	PEDESTRIAN,
	BICYCLE
};

	
enum class RoutingParameterType {
	NUMERIC,
	BOOLEAN,
	SYMBOLIC
};

struct RoutingParameter {
	std::string id;
	std::string name;
	std::string description;
	RoutingParameterType type;
	std::vector<double> possibleValues; // Object TODO;
	std::vector<std::string> possibleValueDescriptions;
};

struct ParameterContext {
	MAP_STR_STR vars;
};

struct RouteAttributeExpression {
	static const int LESS_EXPRESSION;
	static const int GREAT_EXPRESSION;
		
	std::vector<std::string> values;
	int expressionType;
	std::string valueType;
	std::vector<double> cacheValues; 

	RouteAttributeExpression(std::vector<std::string> const & vls, int type, std::string const & vType);
	bool matches(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter * router) const;
	double calculateExprValue(int id, dynbitset const & types, ParameterContext const & paramContext, GeneralRouter * router) const;
};


class RouteAttributeEvalRule {
	friend class RouteAttributeContext;

private: 
	std::vector<std::string> parameters ;
	double selectValue ;
	std::string selectValueDef ;
	std::string selectType;
	dynbitset filterTypes;
	dynbitset filterNotTypes;
		
	UNORDERED(set)<std::string> onlyTags;
	UNORDERED(set)<std::string> onlyNotTags;
	std::vector<RouteAttributeExpression> expressions;

	std::vector<std::string> tagValueCondDefValue;
	std::vector<std::string> tagValueCondDefTag;
	std::vector<bool> tagValueCondDefNot;


	bool matches(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter * router);
	double eval(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter * router);
	double calcSelectValue(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter* router);

	bool checkAllTypesShouldBePresent(dynbitset const & types);
	bool checkAllTypesShouldNotBePresent(dynbitset const & types);
	bool checkNotFreeTags(dynbitset const & types, GeneralRouter * router) const;
	bool checkFreeTags(dynbitset const & types, GeneralRouter * router) const;
	bool checkExpressions(dynbitset const & types, ParameterContext const & paramContext, GeneralRouter* router) const;

	void printRule(GeneralRouter const * r) const;
public:
	void registerAndTagValueCondition(GeneralRouter* r, std::string const & tag, std::string const & value, bool nt);

	// formated as [param1,-param2]
	void registerParamConditions(std::vector<std::string> const & params);
	void registerSelectValue(std::string const & selectValue, std::string const & selectType);
	void registerExpression(RouteAttributeExpression const & expression) {
		expressions.push_back(expression);
	}
};

class RouteAttributeContext {
	friend class GeneralRouter;

private:
	std::vector<RouteAttributeEvalRule> rules;
	ParameterContext paramContext ;
	GeneralRouter* router;

public: 
	RouteAttributeContext(GeneralRouter* r) : router(r) {
	}

	void registerParams(std::vector<std::string> const & keys, std::vector<std::string> const & vls) {
		for(uint i = 0; i < keys.size(); i++) {
			paramContext.vars[keys[i]] = vls[i];
		}
	}

	RouteAttributeEvalRule* newEvaluationRule() {
		RouteAttributeEvalRule c;
		rules.push_back(c);
		return &rules[rules.size() - 1];
	}

	void printRules() const {
		for (uint k = 0; k < rules.size(); k++) {
			RouteAttributeEvalRule const & r = rules[k];
			r.printRule(router);
		}
	}

private:
	double evaluate(dynbitset const & types) {
		for (uint k = 0; k < rules.size(); k++) {
			double o = rules[k].eval(types, paramContext, router);
			if (o != DOUBLE_MISSING) {
				return o;
			}
		}
		return DOUBLE_MISSING;
	}

	dynbitset convert(RoutingIndex* reg, std::vector<uint32_t>& types) const;

	double evaluate(SHARED_PTR<RouteDataObject> const & ro) {
		dynbitset local =  convert(ro->region, ro->types);
		return evaluate(local);
	}

	int evaluateInt(SHARED_PTR<RouteDataObject> const & ro, int defValue) {
		double d = evaluate(ro);
		if(d == DOUBLE_MISSING) {
			return defValue;
		}
		return (int)d;
	}

	double evaluateDouble(RoutingIndex* reg, std::vector<uint32_t>& types, double defValue) {
		dynbitset local =  convert(reg, types);
		double d = evaluate(local);
		if(d == DOUBLE_MISSING) {
			return defValue;
		}
		return d;
	}

	double evaluateDouble(SHARED_PTR<RouteDataObject> ro, double defValue) {
		double d = evaluate(ro);
		if(d == DOUBLE_MISSING) {
			return defValue;
		}
		return d;
	}
};

float parseFloat(MAP_STR_STR & attributes, std::string const & key, float def);
bool parseBool(MAP_STR_STR & attributes, std::string const & key, bool def);
std::string const & parseString(MAP_STR_STR & attributes, std::string const & key, std::string const & def);

class GeneralRouter {
	friend class RouteAttributeContext;
	friend class RouteAttributeEvalRule;
	friend class RouteAttributeExpression;
private:
	std::vector<RouteAttributeContext> objectAttributes;
	MAP_STR_STR attributes;
	UNORDERED(map)<std::string, RoutingParameter> parameters;
	MAP_STR_INT universalRules;
	std::vector<tag_value> universalRulesById;
	UNORDERED(map)<std::string, dynbitset > tagRuleMask;
	std::vector<double> ruleToValue; // Object TODO;
	bool shortestRoute;
	
	///UNORDERED(map)<RoutingIndex*, MAP_INT_INT> regionConvert;
		
public:
	// cached values
	bool _restrictionsAware ;
	double leftTurn;
	double roundaboutTurn;
	double rightTurn;
	double minDefaultSpeed ;
	double maxDefaultSpeed ;

	GeneralRouter() : _restrictionsAware(true), minDefaultSpeed(10), maxDefaultSpeed(10) {

	}

	RouteAttributeContext* newRouteAttributeContext() {
		RouteAttributeContext c(this);
		objectAttributes.push_back(c);
		return &objectAttributes[objectAttributes.size() - 1];
	}

	void addAttribute(std::string const & k, std::string const & v) ;

	bool containsAttribute(std::string const & attribute) const;
	
	std::string const & getAttribute(std::string const & attribute);
	
	/**
	 * return if the road is accepted for routing
	 */
	bool acceptLine(SHARED_PTR<RouteDataObject> const & way);
	
	/**
	 * return oneway +/- 1 if it is oneway and 0 if both ways
	 */
	int isOneWay(SHARED_PTR<RouteDataObject> const & road);
	
	/**
	 * return delay in seconds (0 no obstacles)
	 */
	double defineObstacle(SHARED_PTR<RouteDataObject> const & road, uint point);
	
	/**
	 * return delay in seconds (0 no obstacles)
	 */
	double defineRoutingObstacle(SHARED_PTR<RouteDataObject> const & road, uint point);

	/**
	 * return routing speed in m/s for vehicle for specified road
	 */
	double defineRoutingSpeed(SHARED_PTR<RouteDataObject> const & road);

	/*
	* return transition penalty between different road classes in seconds
	*/
	double definePenaltyTransition(SHARED_PTR<RouteDataObject> const & road);

	/**
	 * return real speed in m/s for vehicle for specified road
	 */
	double defineVehicleSpeed(SHARED_PTR<RouteDataObject> const & road);
	
	/**
	 * define priority to multiply the speed for g(x) A* 
	 */
	double defineSpeedPriority(SHARED_PTR<RouteDataObject> const & road);

	/**
	 * Used for A* routing to calculate g(x)
	 * 
	 * @return minimal speed at road in m/s
	 */
	double getMinDefaultSpeed() const;



	/**
	 * Used for A* routing to predict h(x) : it should be great any g(x)
	 * 
	 * @return maximum speed to calculate shortest distance
	 */
	double getMaxDefaultSpeed() const;
	
	/**
	 * aware of road restrictions
	 */
	bool restrictionsAware() const;
	
	/**
	 * Calculate turn time 
	 */
	double calculateTurnTime(SHARED_PTR<RouteSegment> const & segment, int segmentEnd,
		SHARED_PTR<RouteSegment> const & prev, int prevSegmentEnd);

	void printRules() const {
		for (uint k = 0; k < objectAttributes.size(); k++) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "RouteAttributeContext  %d", k + 1);
			objectAttributes[k].printRules();
		}
	}

private :
	double parseValueFromTag(uint id, std::string const & type, GeneralRouter const * router);
	uint registerTagValueAttribute(const tag_value& r);
	RouteAttributeContext & getObjContext(RouteDataObjectAttribute a) {
		return objectAttributes[(unsigned int)a];
	}

};

#endif /*_OSMAND_GENERAL_ROUTER_H*/
