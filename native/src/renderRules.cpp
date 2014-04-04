#include <iterator>
#include <string>
#include <vector>
#include <stack>
#include <expat.h>
#include "Common.h"
#include "common2.h"
#include "renderRules.h"
#include "Logging.h"

float getDensityValue(RenderingContext const & rc, RenderingRuleSearchRequest const * render, RenderingRuleProperty const * prop) {
	return rc.getDensityValue(render->getFloatPropertyValue(prop, 0),
		render->getIntPropertyValue(prop, 0));
}

/**
 * Parse the color string, and return the corresponding color-int.
 * Supported formats are:
 * #RRGGBB
 * #AARRGGBB
 */
int parseColor(std::string const & colorString) {
	if (colorString[0] == '#') {
		// Use a long to avoid rollovers on #ffXXXXXX
		long color = strtol(colorString.c_str() + 1, NULL, 16);
		if (colorString.size() == 7) {
			// Set the alpha value
			color |= 0x00000000ff000000;
		} else if (colorString.size() != 9) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Unknown color %s", colorString.c_str());
		}
		return (int) color;
	}
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Unknown color %s", colorString.c_str());
	return -1;
}

std::string colorToString(int color) {
	char c[9];
	if ((0xFF000000 & color) == 0xFF000000) {
		sprintf(c, "%x", (color & 0x00FFFFFF));
	} else {
		sprintf(c, "%x", color);
	}
	return std::string(c);
}

RenderingRule::RenderingRule(Attributes const & attrs, RenderingRulesStorage* storage) {
	storage->childRules.push_back(this);
	properties.reserve(attrs.size());
	intProperties.assign(attrs.size(), -1);
	Attributes::const_iterator it = attrs.begin();
	int i = 0;
	for (; it != attrs.end(); it++) {
		RenderingRuleProperty* property = storage->PROPS.getProperty(it->first);
		if (property == NULL) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Property %s was not found in registry", it->first.c_str());
			return ;
		}
		properties.push_back(property);

		if (property->isString()) {
			intProperties[i] = storage->getDictionaryValue(it->second);
		} else if (property->isFloat()) {
			if (floatProperties.size() == 0) {
				// lazy creates
				floatProperties.assign(attrs.size(), -1);
			}
			floatProperties[i] = property->parseFloatValue(it->second);
		} else {
			intProperties[i] = property->parseIntValue(it->second);
		}
		i++;
	}
}

// TODO has "properties" the caracteristics to be a map??
inline int RenderingRule::getPropertyIndex(std::string const & property) const {
		for (int i = properties.size()-1; i >= 0; --i) {
			if (properties[i]->attrName == property) {
				return i;
			}
		}
		return -1;
	}

std::string const & RenderingRule::getStringPropertyValue(std::string const & property,
		RenderingRulesStorage const * storage) const {
	static std::string const empty;
	int i = getPropertyIndex(property);
	if (i >= 0) {
		return storage->getStringValue(intProperties[i]);
	}
	return empty;
}

float RenderingRule::getFloatPropertyValue(std::string const & property) const {
	int i = getPropertyIndex(property);
	if (i >= 0) {
		return floatProperties[i];
	}
	return 0;
}

std::string RenderingRule::getColorPropertyValue(std::string const & property) const {
	int i = getPropertyIndex(property);
	if (i >= 0) {
		return colorToString(intProperties[i]);
	}
	return "";
}

int RenderingRule::getIntPropertyValue(std::string const & property) const {
	int i = getPropertyIndex(property);
	if (i >= 0) {
		return intProperties[i];
	}
	return -1;
}

RenderingRule* RenderingRulesStorage::getRule(int state, int itag, int ivalue) const {
	UNORDERED(map)<int, RenderingRule*>::const_iterator it = (tagValueGlobalRules[state]).find(
			(itag << SHIFT_TAG_VAL) | ivalue);
	if (it == tagValueGlobalRules[state].end()) {
		return NULL;
	}
	return (*it).second;
}

void RenderingRulesStorage::registerGlobalRule(RenderingRule* rr, int state) {
	int tag = rr->getIntPropertyValue(this->PROPS.R_TAG->attrName);
	if (tag == -1) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Attribute tag should be specified for root filter ");
	}
	int value = rr->getIntPropertyValue(this->PROPS.R_VALUE->attrName);
	if (value == -1) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Attribute tag should be specified for root filter ");
	}
	int key = (tag << SHIFT_TAG_VAL) | value;
	RenderingRule* toInsert = rr;
	RenderingRule* previous = tagValueGlobalRules[state][key];

	if (previous != NULL) {
		// all root rules should have at least tag/value
		toInsert = createTagValueRootWrapperRule(key, previous);
		toInsert->ifElseChildren.push_back(rr);
	}
	tagValueGlobalRules[state][key] = toInsert;
}

RenderingRule* RenderingRulesStorage::createTagValueRootWrapperRule(int tagValueKey, RenderingRule* previous) {
	if (previous->properties.size() > 2) {
		Attributes m;
		m["tag"] = getTagString(tagValueKey);
		m["value"] = getValueString(tagValueKey);
		RenderingRule* toInsert = new RenderingRule(m, this);
		toInsert->ifElseChildren.push_back(previous);
		return toInsert;
	} else {
		return previous;
	}
}

struct GroupRules {
	RenderingRule* singleRule;
	Attributes groupAttributes;
	std::vector<RenderingRule*> children;
	std::vector<GroupRules> childrenGroups;

	GroupRules(RenderingRule* singleRule = NULL) : singleRule(singleRule) {
	}

	inline bool isGroup() const {
		return singleRule == NULL;
	}

	void addGroupFilter(RenderingRule* rr) {
		for (std::vector<RenderingRule*>::iterator ch = children.begin(); ch != children.end(); ch++) {
			(*ch)->ifChildren.push_back(rr);
		}
		for (std::vector<GroupRules>::iterator gch = childrenGroups.begin(); gch != childrenGroups.end(); gch++) {
			gch->addGroupFilter(rr);
		}
	}

	void registerGlobalRules(RenderingRulesStorage* storage, int state) {
		for (std::vector<RenderingRule*>::iterator ch = children.begin(); ch != children.end(); ch++) {
			storage->registerGlobalRule(*ch, state);
		}
		for (std::vector<GroupRules>::iterator gch = childrenGroups.begin(); gch != childrenGroups.end(); gch++) {
			gch->registerGlobalRules(storage, state);
		}
	}
};

class RenderingRulesHandler {
	private:
	friend class RenderingRulesStorage;
	int state;
	std::stack<GroupRules> st;
	RenderingRulesStorageResolver const * resolver;
	RenderingRulesStorage* dependsStorage;
	RenderingRulesStorage* storage;

	public:
	RenderingRulesHandler(RenderingRulesStorageResolver const * resolver,
			RenderingRulesStorage * storage) :
			resolver(resolver), dependsStorage(NULL), storage(storage) {
	}

	private:
	RenderingRulesStorage const * getDependsStorage() const {
		return dependsStorage;
	}

	static Attributes& parseAttributes(const char **atts, Attributes& m) {
			//RenderingRulesStorage* st) {
		while (*atts != NULL) {
			// Not used
			//std::string vl = std::string(atts[1]);
			//if(vl.size() > 1 && vl[0] == '$') {
			//	vl = st->renderingConstants[vl.substr(1, vl.size() - 1)];
			//}
			m[std::string(atts[0])] = std::string(atts[1]);
			atts += 2;
		}
		return m;
	}

	public:
	static void startElementHandler(void *data, const char *tag, const char **atts) {
		RenderingRulesHandler* t = (RenderingRulesHandler*) data;
		std::string name(tag);
		if ("filter" == name) {
			Attributes attrsMap;
			if (!t->st.empty() && t->st.top().isGroup()) {
				attrsMap.insert(t->st.top().groupAttributes.begin(), t->st.top().groupAttributes.end());
			}
			parseAttributes(atts, attrsMap);
			RenderingRule* renderingRule = new RenderingRule(attrsMap,t->storage);
			if (!t->st.empty() && t->st.top().isGroup()) {
				t->st.top().children.push_back(renderingRule);
			} else if (!t->st.empty() && !t->st.top().isGroup()) {
				// RenderingRule* parent = t->st.top().singleRule;
				t->st.top().singleRule->ifElseChildren.push_back(renderingRule);
			} else {
				t->storage->registerGlobalRule(renderingRule, t->state);

			}
			t->st.push(GroupRules(renderingRule));
		} else if ("groupFilter" == name) { //$NON-NLS-1$
			Attributes attrsMap;
			parseAttributes(atts, attrsMap);
			RenderingRule* renderingRule = new RenderingRule(attrsMap,t->storage);
			if (!t->st.empty() && t->st.top().isGroup()) {
				t->st.top().addGroupFilter(renderingRule);
			} else if (!t->st.empty() && !t->st.top().isGroup()) {
				t->st.top().singleRule->ifChildren.push_back(renderingRule);
			} else {
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Group filter without parent");
			}
			t->st.push(GroupRules(renderingRule));
		} else if ("group" == name) { //$NON-NLS-1$
			GroupRules groupRules;
			if (!t->st.empty() && t->st.top().isGroup()) {
				groupRules.groupAttributes.insert(t->st.top().groupAttributes.begin(), t->st.top().groupAttributes.end());
			}
			parseAttributes(atts, groupRules.groupAttributes);
			t->st.push(groupRules);
		} else if ("order" == name) { //$NON-NLS-1$
			t->state = RenderingRulesStorage::ORDER_RULES;
		} else if ("text" == name) { //$NON-NLS-1$
			t->state = RenderingRulesStorage::TEXT_RULES;
		} else if ("point" == name) { //$NON-NLS-1$
			t->state = RenderingRulesStorage::POINT_RULES;
		} else if ("line" == name) { //$NON-NLS-1$
			t->state = RenderingRulesStorage::LINE_RULES;
		} else if ("polygon" == name) { //$NON-NLS-1$
			t->state = RenderingRulesStorage::POLYGON_RULES;
		} else if ("renderingConstant" == name) { //$NON-NLS-1$
			//Attributes attrsMap;
			//parseAttributes(atts, attrsMap);
			//t->storage->renderingConstants[attrsMap["name"]] = attrsMap["value"];
		} else if ("renderingAttribute" == name) { //$NON-NLS-1$
			Attributes attrsMap;
			parseAttributes(atts, attrsMap);
			std::string const & attr = attrsMap["name"];
			Attributes empty;
			RenderingRule* root = new RenderingRule(empty,t->storage);
			t->storage->renderingAttributes[attr] = root;
			t->st.push(GroupRules(root));
		} else if ("renderingProperty" == name) {
			Attributes attrsMap;
			parseAttributes(atts, attrsMap);
			std::string const & attr = attrsMap["attr"];
			RenderingRuleProperty* prop;
			std::string const & type = attrsMap["type"];
			if ("boolean" == type) {
				prop = RenderingRuleProperty::createInputBooleanProperty(attr);
			} else if ("string" == type) {
				prop = RenderingRuleProperty::createInputStringProperty(attr);
			} else {
				prop = RenderingRuleProperty::createInputIntProperty(attr);
			}
			//prop->description = attrsMap["description"];
			//prop->name = attrsMap["name"];
			//std::string const & possible = attrsMap["possibleValues"];
			//if (possible != "") {
			//	int n;
			//	int p = 0;
			//	while ((n = possible.find(',', p)) != std::string::npos) {
			//		prop->possibleValues.push_back(possible.substr(p, n));
			//		p = n + 1;
			//	}
			//	prop->possibleValues.push_back(possible.substr(p));
			//}
			t->storage->PROPS.registerRule(prop);
		} else if ("renderingStyle" == name) {
			Attributes attrsMap;
			parseAttributes(atts, attrsMap);
			std::string const & depends = attrsMap["depends"];
			if (depends.size() > 0 && t->resolver != NULL) {
				t->dependsStorage = t->resolver->resolve(depends);
			}
			if (t->dependsStorage != NULL) {
				// copy dictionary
				t->storage->dictionary = t->dependsStorage->dictionary;
				t->storage->dictionaryMap = t->dependsStorage->dictionaryMap;
				t->storage->PROPS.merge(t->dependsStorage->PROPS);
			} else if (depends.size() > 0) {
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "!Dependent rendering style was not resolved : %s", depends.c_str());
			}
		} else {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Unknown tag : %s", name.c_str());
		}
	}

	static void endElementHandler(void *data, const char *tag) {
		RenderingRulesHandler* t = (RenderingRulesHandler*) data;
		std::string name(tag);
		if ("filter" == name) { //$NON-NLS-1$
			t->st.pop();
		} else if ("group" == name) { //$NON-NLS-1$
			GroupRules group = t->st.top();
			t->st.pop();
			if (t->st.empty()) {
				group.registerGlobalRules(t->storage,t->state);
			} else if(t->st.top().isGroup()){
				t->st.top().childrenGroups.push_back(group);
			}
		} else if ("groupFilter" == name) { //$NON-NLS-1$
			t->st.pop();
		} else if ("renderingAttribute" == name) { //$NON-NLS-1$
			t->st.pop();
		}
	}
};

void RenderingRule::printDebugRenderingRule(std::string & indent, RenderingRulesStorage const * st) const {
	indent += "   ";
	printf("\n%s", indent.c_str());
	std::vector<RenderingRuleProperty*>::const_iterator pp = properties.begin();
	for (; pp != properties.end(); pp++) {
		printf(" %s=", (*pp)->attrName.c_str());
		if ((*pp)->isString()) {
			printf("\"%s\"", getStringPropertyValue((*pp)->attrName, st).c_str());
		} else if ((*pp)->isFloat()) {
			printf("%f", getFloatPropertyValue((*pp)->attrName));
		} else if ((*pp)->isColor()) {
			printf("%s", getColorPropertyValue((*pp)->attrName).c_str());
		} else if ((*pp)->isIntParse()) {
			printf("%d", getIntPropertyValue((*pp)->attrName));
		}
	}
	std::vector<RenderingRule*>::const_iterator it = ifElseChildren.begin();
	for (; it != ifElseChildren.end(); it++) {
		std::string cindent = indent + "  + ";
		(*it)->printDebugRenderingRule(cindent, st);
	}
	it = ifChildren.begin();
	for (; it != ifChildren.end(); it++) {
		std::string cindent = indent + "  o  ";
		(*it)->printDebugRenderingRule(cindent, st);
	}
}

void RenderingRulesStorage::printDebug(int state) const {
	UNORDERED(map)<int, RenderingRule*>::const_iterator it = tagValueGlobalRules[state].begin();
	for (; it != tagValueGlobalRules[state].end(); it++) {
		printf("\n\n%s : %s", getTagString(it->first).c_str(), getValueString(it->first).c_str());
		std::string indent;
		it->second->printDebugRenderingRule(indent, this);
	}
}

void RenderingRulesStorage::parseRulesFromXmlInputStream(const char* filename,
		RenderingRulesStorageResolver const * resolver) {
	XML_Parser parser = XML_ParserCreate(NULL);
	RenderingRulesHandler handler = RenderingRulesHandler(resolver, this);
	XML_SetUserData(parser, &handler);
	XML_SetElementHandler(parser, RenderingRulesHandler::startElementHandler, RenderingRulesHandler::endElementHandler);
	FILE *file = fopen(filename, "r");
	if (file == NULL) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "File can not be open %s", filename);
		XML_ParserFree(parser);
		return;
	}
	char buffer[512];
	bool done = false;
	while (!done) {
		fgets(buffer, sizeof(buffer), file);
		int len = strlen(buffer);
		if (feof(file) != 0) {
			done = true;
		}
		if (XML_Parse(parser, buffer, len, done) == XML_STATUS_ERROR) {
			fclose(file);
			XML_ParserFree(parser);
			return;
		}
	}

	RenderingRulesStorage const * depends = handler.getDependsStorage();
	if (depends != NULL) {
		// merge results
		// dictionary and props are already merged
		std::map<std::string, RenderingRule*>::const_iterator it = depends->renderingAttributes.begin();
		for(;it != depends->renderingAttributes.end(); it++) {
			std::map<std::string, RenderingRule*>::iterator o = renderingAttributes.find(it->first);
			if (o != renderingAttributes.end()) {
				std::vector<RenderingRule*>::const_iterator list = it->second->ifElseChildren.begin();
				for (;list != it->second->ifElseChildren.end(); list++) {
					o->second->ifElseChildren.push_back(*list);
				}
			} else {
				renderingAttributes[it->first] = it->second;
			}
		}

		for (int i = 0; i < SIZE_STATES; i++) {
			if (depends->tagValueGlobalRules[i].empty()) {
				continue;
			}
			UNORDERED(map)<int, RenderingRule*>::const_iterator it = depends->tagValueGlobalRules[i].begin();
			for (; it != depends->tagValueGlobalRules[i].end(); it++) {
				UNORDERED(map)<int, RenderingRule*>::iterator o = tagValueGlobalRules[i].find(it->first);
				RenderingRule* toInsert = it->second;
				if (o != tagValueGlobalRules[i].end()) {
					toInsert = createTagValueRootWrapperRule(it->first, o->second);
					toInsert->ifElseChildren.push_back(it->second);
				}
				tagValueGlobalRules[i][it->first] = toInsert;
			}
		}
	}
	XML_ParserFree(parser);
	fclose(file);
}

RenderingRuleSearchRequest::RenderingRuleSearchRequest(RenderingRulesStorage* storage)  {
	this->storage = storage;
	PROPS = &(this->storage->PROPS);
	this->values.resize(PROPS->properties.size(), 0);
	this->fvalues.resize(PROPS->properties.size(), 0);
	UNORDERED(map)<std::string, RenderingRuleProperty*>::const_iterator it = PROPS->properties.begin();
	for (; it != PROPS->properties.end(); it++) {
		if (!it->second->isColor()) {
			values[it->second->id] = -1;
		}
	}
	setBooleanFilter(PROPS->R_TEST, true);
	saveState();
}

void RenderingRuleSearchRequest::saveState() {
	this->savedFvalues = fvalues;
	this->savedValues = values;
}

RenderingRuleSearchRequest::~RenderingRuleSearchRequest() {
}

int RenderingRuleSearchRequest::getIntPropertyValue(RenderingRuleProperty const * prop) const {
	if (prop == NULL) {
		return 0;
	}
	return values[prop->id];
}

int RenderingRuleSearchRequest::getIntPropertyValue(RenderingRuleProperty const * prop, int def) const {
	if (prop == NULL || values[prop->id] == -1) {
		return def;
	}
	return values[prop->id];
}

std::string const & RenderingRuleSearchRequest::getStringPropertyValue(RenderingRuleProperty const * prop) const {
	static std::string const empty;
	if (prop == NULL) {
		return empty;
	}
	int s = values[prop->id];
	return storage->getDictionaryValue(s);
}

float RenderingRuleSearchRequest::getFloatPropertyValue(RenderingRuleProperty const * prop, float def) const {
	if (prop == NULL || fvalues[prop->id] == 0) {
		return def;
	}
	return fvalues[prop->id];
}

void RenderingRuleSearchRequest::setStringFilter(RenderingRuleProperty const * p, std::string const & filter) {
	if (p != NULL) {
		values[p->id] = storage->getDictionaryValue(filter);
	}
}

void RenderingRuleSearchRequest::setIntFilter(RenderingRuleProperty const * p, int filter) {
	if (p != NULL) {
		values[p->id] = filter;
	}
}

void RenderingRuleSearchRequest::setBooleanFilter(RenderingRuleProperty const * p, bool filter) {
	if (p != NULL) {
		values[p->id] = filter ? TRUE_VALUE : FALSE_VALUE;
	}
}

void RenderingRuleSearchRequest::clearIntvalue(RenderingRuleProperty const * p) {
	if (p != NULL) {
		values[p->id] = -1;
	}
}

void RenderingRuleSearchRequest::externalInitialize(
		std::vector<int> const & vs, std::vector<float> const & fvs,
		std::vector<int> const & sVs, std::vector<float> const & sFvs) {
	this->values = vs;
	this->fvalues = fvs;
	this->savedValues = sVs;
	this->savedFvalues = sFvs;
}

bool RenderingRuleSearchRequest::searchRule(int state) {
	return search(state, true);
}

bool RenderingRuleSearchRequest::searchRenderingAttribute(std::string const & attribute) {
	RenderingRule const * rule = storage->getRenderingAttributeRule(attribute);
	if (rule == NULL) {
		return false;
	}
	return visitRule(rule, true);
}

bool RenderingRuleSearchRequest::search(int state, bool loadOutput) {
	int tagKey = values[PROPS->R_TAG->id];
	int valueKey = values[PROPS->R_VALUE->id];
	// Short circuit logic
	return searchInternal(state, tagKey, valueKey, loadOutput)
			|| searchInternal(state, tagKey, 0, loadOutput)
			|| searchInternal(state, 0, 0, loadOutput);
}

bool RenderingRuleSearchRequest::searchInternal(int state, int tagKey, int valueKey, bool loadOutput) {
	values[PROPS->R_TAG->id] = tagKey;
	values[PROPS->R_VALUE->id] = valueKey;
	values[PROPS->R_DISABLE->id] = 0;
	RenderingRule* accept = storage->getRule(state, tagKey, valueKey);
	if (accept == NULL) {
		return false;
	}
	bool match = visitRule(accept, loadOutput);
	if(match && values[PROPS->R_DISABLE->id] != 0) {
		return false;
	}
	return match;
}

bool RenderingRuleSearchRequest::visitRule(RenderingRule const * rule, bool loadOutput) {
	std::vector<RenderingRuleProperty*> const & properties = rule->properties;
	int propLen = rule->properties.size();
	for (int i = 0; i < propLen; i++) {
		RenderingRuleProperty const * rp = properties[i];
		if (rp != NULL && rp->input) {
			bool match = false;
			if (rp->isFloat()) {
				match = rule->floatProperties[i] == fvalues[rp->id];
			} else if (rp == PROPS->R_MINZOOM) {
				match = rule->intProperties[i] <= values[rp->id];
			} else if (rp == PROPS->R_MAXZOOM) {
				match = rule->intProperties[i] >= values[rp->id];
			} else if (rp == PROPS->R_ADDITIONAL) {
				if (obj == NULL) {
					match = true;
				} else {
					std::string const & val = storage->getDictionaryValue(rule->intProperties[i]);
					int i = val.find('=');
					if (i >= 0) {
						match = obj->containsAdditional(val.substr(0, i), val.substr(i + 1));
					} else {
						match = false;
					}
				}
			} else {
				match = rule->intProperties[i] == values[rp->id];
			}
			if (!match) {
				return false;
			}
		} else if (rp == PROPS->R_DISABLE) {
			values[PROPS->R_DISABLE->id] = rule->intProperties[i];
		}
	}
	if (!loadOutput) {
		return true;
	}
	// accept it
	for (int i = 0; i < propLen; i++) {
		RenderingRuleProperty const * rp = properties[i];
		if (rp != NULL && !rp->input) {
			//searchResult = true;
			if (rp->isFloat()) {
				fvalues[rp->id] = rule->floatProperties[i];
				values[rp->id] = rule->intProperties[i];
			} else {
				values[rp->id] = rule->intProperties[i];
			}
		}
	}
	size_t j;
	for (j = 0; j < rule->ifElseChildren.size(); j++) {
		bool match = visitRule(rule->ifElseChildren[j], loadOutput);
		if (match) {
			break;
		}
	}
	for (j = 0; j < rule->ifChildren.size(); j++) {
		visitRule(rule->ifChildren[j], loadOutput);
	}
	return true;
}

void RenderingRuleSearchRequest::clearState() {
	obj = NULL;
	values = savedValues;
	fvalues = savedFvalues;
}

void RenderingRuleSearchRequest::setInitialTagValueZoom(std::string const & tag, std::string const & value,
		int zoom, MapDataObject* obj) {
	clearState();
	this->obj = obj;
	setIntFilter(PROPS->R_MINZOOM, zoom);
	setIntFilter(PROPS->R_MAXZOOM, zoom);
	setStringFilter(PROPS->R_TAG, tag);
	setStringFilter(PROPS->R_VALUE, value);
}

void RenderingRuleSearchRequest::setTagValueZoomLayer(std::string const & tag, std::string const & val,
		int zoom, int layer, MapDataObject* obj) {
	this->obj = obj;
	setIntFilter(PROPS->R_MINZOOM, zoom);
	setIntFilter(PROPS->R_MAXZOOM, zoom);
	setIntFilter(PROPS->R_LAYER, layer);
	setStringFilter(PROPS->R_TAG, tag);
	setStringFilter(PROPS->R_VALUE, val);
}

bool RenderingRuleSearchRequest::isSpecified(RenderingRuleProperty const * p) const {
	if (p->isFloat()) {
		return fvalues[p->id] != 0 || values[p->id] != -1;
	} else {
		int val = values[p->id];
		if (p->isColor()) {
			return val != 0;
		} else {
			return val != -1;
		}
	}
}
