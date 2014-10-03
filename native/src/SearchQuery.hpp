/*
 * SearchQuery.hpp
 *
 *  Created on: 09/09/2014
 *      Author: javier
 */

#ifndef SEARCHQUERY_HPP_
#define SEARCHQUERY_HPP_

#include "renderRules.h"

struct ResultPublisher {
	std::vector< MapDataObject*> result;

	bool publish(MapDataObject* r) {
		result.push_back(r);
		return true;
	}
	bool publish(std::vector<MapDataObject*> const & r) {
		result.insert(result.begin(), r.begin(), r.end());
		return true;
	}
	bool isCancelled() const {
		return false;
	}
	virtual ~ResultPublisher() {
		//deleteObjects(result);
	}
};

struct SearchQuery {
	RenderingRuleSearchRequest* req;
	uint32_t left;
	uint32_t right;
	uint32_t top;
	uint32_t bottom;
	int zoom;
	ResultPublisher* publisher;

	bool ocean;
	bool mixed;

	uint numberOfVisitedObjects;
	uint numberOfAcceptedObjects;
	uint numberOfReadSubtrees;
	uint numberOfAcceptedSubtrees;

	SearchQuery(int l, int r, int t, int b, RenderingRuleSearchRequest* req, ResultPublisher* publisher) :
			req(req), left(l), right(r), top(t), bottom(b),publisher(publisher) {
		numberOfAcceptedObjects = numberOfVisitedObjects = 0;
		numberOfAcceptedSubtrees = numberOfReadSubtrees = 0;
		ocean = mixed = false;
	}
	SearchQuery(int l, int r, int t, int b) :
				left(l), right(r), top(t), bottom(b) {
	}

	SearchQuery(){

	}

	bool publish(MapDataObject* obj) {
		return publisher->publish(obj);
	}

	bool acceptTypes(std::vector<tag_value> const & types)
	{
		for (std::vector<tag_value>::const_iterator type = types.begin(); type != types.end(); type++)
		{
			req->setIntFilter(req->props()->R_MINZOOM, zoom);
			req->setStringFilter(req->props()->R_TAG, type->first);
			req->setStringFilter(req->props()->R_VALUE, type->second);
			for (int i = 1; i <= 3; i++)
				if (req->search(i, false))
					return true;
			req->setStringFilter(req->props()->R_NAME_TAG, "");
			if (req->search(RenderingRulesStorage::TEXT_RULES, false))
				return true;
		}

		return false;
	}
};

#endif /* SEARCHQUERY_HPP_ */
