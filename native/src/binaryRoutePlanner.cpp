#include "Common.h"
#include "common2.h"
#include <queue>
#include "binaryRoutePlanner.h"

#include "Logging.h"

static const int REVERSE_WAY_RESTRICTION_ONLY = 1024;

static const int ROUTE_POINTS = 11;
static const short RESTRICTION_NO_RIGHT_TURN = 1;
static const short RESTRICTION_NO_LEFT_TURN = 2;
static const short RESTRICTION_NO_U_TURN = 3;
static const short RESTRICTION_NO_STRAIGHT_ON = 4;
static const short RESTRICTION_ONLY_RIGHT_TURN = 5;
static const short RESTRICTION_ONLY_LEFT_TURN = 6;
static const short RESTRICTION_ONLY_STRAIGHT_ON = 7;
static const bool TRACE_ROUTING = false;

inline int roadPriorityComparator(float o1DistanceFromStart, float o1DistanceToEnd, float o2DistanceFromStart,
		float o2DistanceToEnd, float heuristicCoefficient) {
	// f(x) = g(x) + h(x)  --- g(x) - distanceFromStart, h(x) - distanceToEnd (not exact)
	float f1 = o1DistanceFromStart + heuristicCoefficient * o1DistanceToEnd;
	float f2 = o2DistanceFromStart + heuristicCoefficient * o2DistanceToEnd;
	if (f1 == f2) {
		return 0;
	}
	return f1 < f2 ? -1 : 1;
}

void printRoad(const char* prefix, SHARED_PTR<RouteSegment> const & segment) {
       OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "%s Road id=%lld dir=%d ind=%d ds=%f es=%f pend=%d parent=%lld",
               prefix, segment->road->id, 
               segment->getSegmentStart(),
               segment->distanceFromStart, segment->distanceToEnd, 
               segment->parentRoute != NULL? segment->parentSegmentEnd : 0,
               segment->parentRoute != NULL? segment->parentRoute->road->id : 0);
}

static double h(RoutingContext* ctx, float distanceToFinalPoint, SHARED_PTR<RouteSegment> const & next) {
	return distanceToFinalPoint / ctx->config.router.getMaxDefaultSpeed();
}

// translate into meters
static double squareRootDist(int x1, int y1, int x2, int y2) {
       double dy = convert31YToMeters(y1, y2);
       double dx = convert31XToMeters(x1, x2);
       return sqrt(dx * dx + dy * dy);
//             return measuredDist(x1, y1, x2, y2);
}

std::pair<int, int> getProjectionPoint(int px, int py, int xA, int yA, int xB, int yB) {
       double mDist = squareRootDist(xA,yA, xB,yB);
       int prx = xA;
       int pry = yA;
       double projection = calculateProjection31TileMetric(xA, yA, xB, yB, px, py);
       if (projection < 0) {
               prx = xA;
               pry = yA;
       } else if (projection >= mDist * mDist) {
               prx = xB;
               pry = yB;
       } else {
               double c = projection / (mDist * mDist);
               prx = (int) ((double)xA + ((double)xB - xA) * c);
               pry = (int) ((double)yA + ((double)yB - yA) * c);
       }
       return std::pair<int, int> (prx, pry);
}

int64_t calculateRoutePointId(SHARED_PTR<RouteDataObject> road, int intervalId, bool positive) {
       return (road->id << ROUTE_POINTS) + (intervalId << 1) + (positive ? 1 : 0);
}

int64_t calculateRoutePointId(SHARED_PTR<RouteSegment> segm, bool direction) {
       if(segm->getSegmentStart() == 0 && !direction) {
               OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Assert failed route point id  0");
       }
       if(segm->getSegmentStart() == segm->getRoad()->getPointsLength() - 1 && direction) {
               OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Assert failed route point length");
       }
       return calculateRoutePointId(segm->getRoad(),
                               direction ? segm->getSegmentStart() : segm->getSegmentStart() - 1, direction);
}

float PrecalculatedRouteDirection::getDeviationDistance(int x31, int y31, int ind) {
	float distToPoint = 0; //squareRootDist(x31, y31, pointsX.get(ind), pointsY.get(ind));
	if(ind < (int)pointsX.size() - 1 && ind != 0) {
		double nx = squareRootDist(x31, y31, pointsX[ind + 1], pointsY[ind + 1]);
		double pr = squareRootDist(x31, y31, pointsX[ind - 1], pointsY[ind - 1]);
		int nind =  nx > pr ? ind -1 : ind +1;
		std::pair<int, int> proj = getProjectionPoint(x31, y31, pointsX[ind], pointsY[ind], pointsX[nind], pointsX[nind]);
		distToPoint = (float) squareRootDist(x31, y31, (int)proj.first, (int)proj.second) ;
	}
	return distToPoint;
}

int PrecalculatedRouteDirection::SHIFT = (1 << (31 - 17));
int PrecalculatedRouteDirection::SHIFTS[] = {1 << (31 - 15), 1 << (31 - 13), 1 << (31 - 12), 
		1 << (31 - 11), 1 << (31 - 7)};
int PrecalculatedRouteDirection::getIndex(int x31, int y31) {
	int ind = -1;
	std::vector<int> cachedS;
	SkRect rct = SkRect::MakeLTRB(x31 - SHIFT, y31 - SHIFT, x31 + SHIFT, y31 + SHIFT);
	quadTree.query_in_box(rct, cachedS);
	if (cachedS.size() == 0) {
		for (uint k = 0; k < 5 /* SHIFTS.size()*/; k++) {
			rct = SkRect::MakeLTRB(x31 - SHIFTS[k], y31 - SHIFTS[k], x31 + SHIFTS[k], y31 + SHIFTS[k]);
			quadTree.query_in_box(rct, cachedS);
			if (cachedS.size() != 0) {
				break;
			}
		}
		if (cachedS.size() == 0) {
			return -1;
		}
	}
	double minDist = 0;
	for (uint i = 0; i < cachedS.size(); i++) {
		int n = cachedS[i];
		double ds = squareRootDist(x31, y31, pointsX[n], pointsY[n]);
		if (ds < minDist || i == 0) {
			ind = n;
			minDist = ds;
		}
	}
	return ind;
}

float PrecalculatedRouteDirection::timeEstimate(int sx31, int sy31, int ex31, int ey31) {
	uint64_t l1 = calc(sx31, sy31);
	uint64_t l2 = calc(ex31, ey31);
	int x31 = sx31;
	int y31 = sy31;
	bool start = false;
	if(l1 == startPoint || l1 == endPoint) {
		start = l1 == startPoint;
		x31 = ex31;
		y31 = ey31;
	} else if(l2 == startPoint || l2 == endPoint) {
		start = l2 == startPoint;
		x31 = sx31;
		y31 = sy31;
	} else {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "! Alert unsupported time estimate ");
		return -2;
	}
	int ind = getIndex(x31, y31);
	if(ind == -1) {
		return -1;
	}
	if((ind == 0 && start) || 
			(ind == (int)pointsX.size() - 1 && !start)) {
		return -1;
	}
	float distToPoint = getDeviationDistance(x31, y31, ind);
	float deviationPenalty = distToPoint / minSpeed;
    float finishTime = (start? startFinishTime : endFinishTime);
	if(start) {
		return (times[0] - times[ind]) +  deviationPenalty + finishTime;
	} else {
		return times[ind] + deviationPenalty + finishTime;
	}
}

/****
static double h(RoutingContext* ctx, int begX, int begY, int endX, int endY) {
	double distToFinalPoint = squareRootDist(begX, begY,  endX, endY);
	double result = distToFinalPoint /  ctx->config.router.getMaxDefaultSpeed();
	if(!ctx->precalcRoute.empty){
		float te = ctx->precalcRoute.timeEstimate(begX, begY,  endX, endY);
		if(te > 0) return te;
	}
	return result;
}
***/
static double h(RoutingContext* ctx, int targetEndX, int targetEndY, int startX, int startY) {
	double distance = distance31TileMetric(startX, startY, targetEndX, targetEndY);
	return distance / ctx->config.router.getMaxDefaultSpeed();
}

struct SegmentsComparator
		: public std::binary_function<SHARED_PTR<RouteSegment>, SHARED_PTR<RouteSegment>, bool>
{
private:
	RoutingContext* ctx;

public:
	SegmentsComparator(RoutingContext* c) : ctx(c) {
	}
	inline bool operator()(const SHARED_PTR<RouteSegment> & lhs, const SHARED_PTR<RouteSegment> & rhs) const
	{
		return roadPriorityComparator(lhs->distanceFromStart, lhs->distanceToEnd,
				rhs->distanceFromStart, rhs->distanceToEnd, ctx->getHeuristicCoefficient()) > 0;
	}
};

struct NonHeuristicSegmentsComparator
		: public std::binary_function<SHARED_PTR<RouteSegment>, SHARED_PTR<RouteSegment>, bool>
{
	inline bool operator()(const SHARED_PTR<RouteSegment> & lhs, const SHARED_PTR<RouteSegment> & rhs) const
	{
		return roadPriorityComparator(lhs->distanceFromStart, lhs->distanceToEnd,
				rhs->distanceFromStart, rhs->distanceToEnd, 0.5) > 0;
	}
};

typedef UNORDERED(map)<int64_t, SHARED_PTR<RouteSegment> > VISITED_MAP;
typedef std::priority_queue<SHARED_PTR<RouteSegment>, std::vector<SHARED_PTR<RouteSegment> >, SegmentsComparator > SEGMENTS_QUEUE;

bool processRouteSegment(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
		VISITED_MAP& visitedSegments, int targetEndX, int targerEndY, SHARED_PTR<RouteSegment> const & segment,
		VISITED_MAP& oppositeSegments);

bool processIntersections(RoutingContext* ctx, SEGMENTS_QUEUE& graphSegments, VISITED_MAP const & visitedSegments,
		VISITED_MAP const & oppositeSegments, double distFromStart, double distToFinalPoint,
		SHARED_PTR<RouteSegment> const & segment, int segmentEnd, SHARED_PTR<RouteSegment> const & inputNext,
		bool reverseWay);

int calculateSizeOfSearchMaps(SEGMENTS_QUEUE const & graphDirectSegments,
		SEGMENTS_QUEUE const & graphReverseSegments,
		VISITED_MAP const & visitedDirectSegments, VISITED_MAP const & visitedOppositeSegments)
{
	int sz = visitedDirectSegments.size() * sizeof(std::pair<int64_t, SHARED_PTR<RouteSegment> > );
	sz += visitedOppositeSegments.size()*sizeof(std::pair<int64_t, SHARED_PTR<RouteSegment> >);
	sz += graphDirectSegments.size()*sizeof(SHARED_PTR<RouteSegment>);
	sz += graphReverseSegments.size()*sizeof(SHARED_PTR<RouteSegment>);
	return sz;
}

/**
 * Calculate route between start.segmentEnd and end.segmentStart (using A* algorithm)
 */
void searchRouteInternal(RoutingContext* ctx,
		SHARED_PTR<RouteSegment> const & start, SHARED_PTR<RouteSegment> const & end,
		bool leftSideNavigation) {
	// FIXME intermediate points
	// measure time
	ctx->visitedSegments = 0;
	int iterationsToUpdate = 0;
	ctx->timeToCalculate.Start();
	if(ctx->config.initialDirection > -180 && ctx->config.initialDirection < 180) {
		ctx->firstRoadId = (start->road->id << ROUTE_POINTS) + start->getSegmentStart();
		double plusDir = start->road->directionRoute(start->getSegmentStart(), true);
		double diff = plusDir - ctx->config.initialDirection;
		if(abs(alignAngleDifference(diff)) <= M_PI / 3) {
			ctx->firstRoadDirection = 1;
		} else if(abs(alignAngleDifference(diff - M_PI )) <= M_PI / 3) {
			ctx->firstRoadDirection = -1;
		}
	}

	SegmentsComparator sgmCmp(ctx);
	SEGMENTS_QUEUE graphDirectSegments(sgmCmp);
	SEGMENTS_QUEUE graphReverseSegments(sgmCmp);

	// Set to not visit one segment twice (stores road.id << X + segmentStart)
	VISITED_MAP visitedDirectSegments;
	VISITED_MAP visitedOppositeSegments;

	// FIXME run recalculation
	bool runRecalculation = false;

	// for start : f(start) = g(start) + h(start) = 0 + h(start) = h(start)
	int targetEndX = end->road->pointsX[end->segmentStart];
	int targetEndY = end->road->pointsY[end->segmentStart];
	int startX = start->road->pointsX[start->segmentStart];
	int startY = start->road->pointsY[start->segmentStart];
	float estimatedDistance = (float) h(ctx, targetEndX, targetEndY, startX, startY);
	end->distanceToEnd = start->distanceToEnd = estimatedDistance;

	graphDirectSegments.push(start);
	graphReverseSegments.push(end);

	// Extract & analyze segment with min(f(x)) from queue while final segment is not found
	bool inverse = false;
	bool init = false;

	NonHeuristicSegmentsComparator nonHeuristicSegmentsComparator;
	SEGMENTS_QUEUE * graphSegments = &graphDirectSegments;

	while (!graphSegments->empty()) {
		SHARED_PTR<RouteSegment> segment = graphSegments->top();
		graphSegments->pop();
		bool routeFound = false;
		if (!inverse) {
			routeFound = processRouteSegment(ctx, false, graphDirectSegments, visitedDirectSegments,
					targetEndX, targetEndY,
					segment, visitedOppositeSegments);
		} else {
			routeFound = processRouteSegment(ctx, true, graphReverseSegments, visitedOppositeSegments,
					startX, startY, segment,
					visitedDirectSegments);
		}
		if (ctx->progress != NULL && iterationsToUpdate-- < 0) {
			iterationsToUpdate = 100;
			ctx->progress->updateStatus(graphDirectSegments.empty()? 0 :graphDirectSegments.top()->distanceFromStart,
					graphDirectSegments.size(),
					graphReverseSegments.empty()? 0 :graphReverseSegments.top()->distanceFromStart,
					graphReverseSegments.size());
			if(ctx->progress->isCancelled()) {
				break;
			}
		}
		if (graphReverseSegments.empty() || graphDirectSegments.empty() || routeFound) {
			break;
		}
		if(runRecalculation) {
			// nothing to do
			inverse = false;
		} else if (!init) {
			inverse = !inverse;
			init = true;
		} else if (ctx->planRouteIn2Directions()) {
			inverse = !nonHeuristicSegmentsComparator(graphDirectSegments.top(), graphReverseSegments.top());
			if (graphDirectSegments.size() * 1.3 > graphReverseSegments.size()) {
				inverse = true;
			} else if (graphDirectSegments.size() < 1.3 * graphReverseSegments.size()) {
				inverse = false;
			}
		} else {
			// different strategy : use onedirectional graph
			inverse = ctx->getPlanRoadDirection() < 0;
		}
		if (inverse) {
			graphSegments = &graphReverseSegments;
		} else {
			graphSegments = &graphDirectSegments;
		}

		// check if interrupted
		if(ctx->isInterrupted()) {
			return;
		}
	}
	ctx->timeToCalculate.Pause();
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "[Native] Result visited (visited roads %d, visited segments %d / %d , queue sizes %d / %d ) ",
			ctx-> visitedSegments, visitedDirectSegments.size(), visitedOppositeSegments.size(),
			graphDirectSegments.size(),graphReverseSegments.size());
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "[Native] Result timing (time to load %d, time to calc %d, loaded tiles %d) ",
			ctx->timeToLoad.GetElapsedMs(), ctx->timeToCalculate.GetElapsedMs(), ctx->loadedTiles);
	int sz = calculateSizeOfSearchMaps(graphDirectSegments, graphReverseSegments, visitedDirectSegments, visitedOppositeSegments);
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "[Native] Memory occupied (Routing context %d Kb, search %d Kb)", ctx->getSize()/1024, sz/1024);
}

bool visitRouteSegment(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
		VISITED_MAP& visitedSegments, int targetEndX, int targetEndY, SHARED_PTR<RouteSegment> const & segment,
		VISITED_MAP const & oppositeSegments, int delta, double obstacleTime);
SHARED_PTR<RouteSegment> proccessRestrictions(RoutingContext* ctx, SHARED_PTR<RouteDataObject> const & road,
		SHARED_PTR<RouteSegment> const & inputNext, bool reverseWay);

bool processRouteSegment(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
		VISITED_MAP& visitedSegments, int targetEndX, int targetEndY, SHARED_PTR<RouteSegment> const & segment,
		VISITED_MAP& oppositeSegments)
{
	// 0. Skip previously visited points
	// TODO Maybe const
	SHARED_PTR<RouteDataObject> const & road = segment->road;
	int start = segment->segmentStart;
	int64_t nt = (road->id << ROUTE_POINTS) + start;
	if (visitedSegments.count(nt) != 0)
	{
		return false;
	}

	// 1. mark route segment as visited
	ctx->visitedSegments++;
	// Route thru segment
	visitedSegments[nt] = segment;

	int roadDirection = ctx->config.router.isOneWay(road);

	if (TRACE_ROUTING)
	{
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
				"Process segment (%d{%d}, %d) name=%s dist=%f roadDirection=%d",
				road->id, road->pointsX.size(), start,
				road->getName().c_str(), segment->distanceFromStart, roadDirection);
	}

	if ( ((!reverseWaySearch && roadDirection >= 0)	|| (reverseWaySearch && roadDirection <= 0))
			&& start < (road->pointsX.size()-1) )
	{
		// We have bigger indexes to visit.
		// We penalize if trying the reverse direction.
		double obstacleTime = (ctx->firstRoadId == nt && ctx->firstRoadDirection < 0)?500:0;
		if (segment->parentRoute != NULL)
		{
			obstacleTime = ctx->config.router.calculateTurnTime(segment, road->pointsX.size()-1,
					segment->parentRoute, segment->parentSegmentEnd);
		}
		if (visitRouteSegment(ctx, reverseWaySearch,
				graphSegments, visitedSegments, targetEndX, targetEndY,
				segment, oppositeSegments, 1, obstacleTime) ) return true;
	}
	if ( ((!reverseWaySearch && roadDirection <= 0)	|| (reverseWaySearch && roadDirection >= 0))
				&& start > 0 )
	{
		// We have smaller indexes to visit
		// We penalize if trying the reverse direction.
		double obstacleTime = (ctx->firstRoadId == nt && ctx->firstRoadDirection > 0)?500:0;
		if (segment->parentRoute != NULL)
		{
			obstacleTime = ctx->config.router.calculateTurnTime(segment, 0,
					segment->parentRoute, segment->parentSegmentEnd);
		}
		if (visitRouteSegment(ctx, reverseWaySearch,
						graphSegments, visitedSegments, targetEndX, targetEndY,
						segment, oppositeSegments, -1, obstacleTime) ) return true;
	}
	return false;
}

bool visitRouteSegment(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE & graphSegments,
		VISITED_MAP & visitedSegments, int targetEndX, int targetEndY, SHARED_PTR<RouteSegment> const & segment,
		VISITED_MAP const & oppositeSegments, int delta, double obstacleTime)
{
	// TODO Maybe const
	SHARED_PTR<RouteDataObject> const & road = segment->road;
	int start = segment->segmentStart + delta;
	int end = (delta == 1)?road->pointsX.size():-1;
	// ! Actually there is small bug when there is restriction to move forward on way (it doesn't take into account)
	double distOnRoadToPass = 0;
	while (start != end)
	{
		// algorithm should visit all reacheable points on the road
		int64_t nts = (road->id << ROUTE_POINTS) + start;
		// Only visited
		visitedSegments[nts] = NULL;

		// 2. calculate point and try to load neighbor ways if they are not loaded
		int x = road->pointsX[start];
		int y = road->pointsY[start];
		distOnRoadToPass += distance31TileMetric(x, y,
				road->pointsX[start-delta], road->pointsY[start-delta]);

		// 2.1 check possible obstacle plus time
		double obstacle = ctx->config.router.defineRoutingObstacle(road, start);
		if (obstacle < 0) continue;
		obstacleTime += obstacle;

		// could be expensive calculation
		SHARED_PTR<RouteSegment> next = ctx->loadRouteSegment(x, y);
		// Be aware of restrictions
		next = proccessRestrictions(ctx, segment->road, next, reverseWaySearch);
		// 3. get intersected ways
		if (next != NULL) {
/*** TODO extract solution check code. Solution point match this same condition. We need to check for solution.
			if (next->next == NULL && next->road->id == road->id)
			{
				// simplification if there is no real intersection
				continue;
			}
***/

			// Using A* routing algorithm
			// g(x) - calculate distance to that point and calculate time
			double priority = ctx->config.router.defineSpeedPriority(road);
			double speed = ctx->config.router.defineRoutingSpeed(road) * priority;
			if (speed == 0) {
				speed = ctx->config.router.getMinDefaultSpeed() * priority;
			}
			double distStartObstacles = segment->distanceFromStart + obstacleTime + distOnRoadToPass / speed;

			if (TRACE_ROUTING)
			{
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
						">> ?%d distToPass=%f speed=%f prio=%f time=%f start=%f",
						start, distOnRoadToPass, ctx->config.router.defineRoutingSpeed(road), priority,
					distOnRoadToPass/speed, segment->distanceFromStart);
			}

			double distToFinalPoint = distance31TileMetric(x, y, targetEndX, targetEndY);
			if (processIntersections(ctx, graphSegments, visitedSegments, oppositeSegments,
					distStartObstacles, distToFinalPoint, segment, start, next, reverseWaySearch) ) return true;
		}  // end of next != NULL
	// next
		start += delta;
	}
	return false;
}

// TODO Las restricciones se tienen en cuenta dependiendo del orden. Hay que revisar el algoritmo.
// Buscar un grano más fino en su tratamiento. Crear indice?
SHARED_PTR<RouteSegment> proccessRestrictions(RoutingContext* ctx, SHARED_PTR<RouteDataObject> const & road,
		SHARED_PTR<RouteSegment> const & inputNext, bool reverseWay)
{
	if (!reverseWay && road->restrictions.empty()) {
		return inputNext;
	}
	if(!ctx->config.router.restrictionsAware()) {
		return inputNext;
	}

	bool exclusiveRestriction = false;
	std::vector<SHARED_PTR<RouteSegment> > segmentsToVisitPrescripted;
	std::vector<SHARED_PTR<RouteSegment> > segmentsToVisitNotForbidden;
	SHARED_PTR<RouteSegment> next = inputNext;
	while (next != NULL) {
		int type = -1;
		if (!reverseWay)
		{
			// TODO mapa para las restricciones
			int64_t id = next->road->id;
			for (size_t i = 0; i < road->restrictions.size(); i++) {
				if ((road->restrictions[i] >> 3) == id) {
					type = road->restrictions[i] & 7;
					break;
				}
			}
		}
		else
		{
			for (size_t i = 0; i < next->road->restrictions.size(); i++) {
				int rt = next->road->restrictions[i] & 7;
				int64_t restrictedTo = next->road->restrictions[i] >> 3;
				if (restrictedTo == road->id) {
					type = rt;
					break;
				}
				// Check if there is restriction only to the other than current road
				if (rt == RESTRICTION_ONLY_RIGHT_TURN || rt == RESTRICTION_ONLY_LEFT_TURN
				|| rt == RESTRICTION_ONLY_STRAIGHT_ON) {
					// check if that restriction applies to considered junk
					SHARED_PTR<RouteSegment> foundNext = inputNext;
					while (foundNext != NULL) {
						if (foundNext->road->id == restrictedTo) {
							break;
						}
						foundNext = foundNext->next;
					}
					if (foundNext != NULL) {
						type = REVERSE_WAY_RESTRICTION_ONLY; // special constant
					}
				}
			}
		}
		if (type != -1 || exclusiveRestriction)
		{
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "type=%d exclusive=%d %d%s%d",
					type, exclusiveRestriction, road->id, reverseWay?"<-":"->", next->road->id);
		}
		if (type == REVERSE_WAY_RESTRICTION_ONLY) {
			// next = next.next; continue;
		} else if (type == -1 && exclusiveRestriction) {
			// next = next.next; continue;
		} else if (type == RESTRICTION_NO_LEFT_TURN || type == RESTRICTION_NO_RIGHT_TURN
				|| type == RESTRICTION_NO_STRAIGHT_ON || type == RESTRICTION_NO_U_TURN) {
			// next = next.next; continue;
		} else if (type == -1) {
			// case no restriction
			segmentsToVisitNotForbidden.push_back(next);
		} else {
			// case exclusive restriction (only_right, only_straight, ...)
			// 1. in case we are going backward we should not consider only_restriction
			// as exclusive because we have many "in" roads and one "out"
			// 2. in case we are going forward we have one "in" and many "out"
			if (!reverseWay) {
				exclusiveRestriction = true;
				segmentsToVisitNotForbidden.clear();
				segmentsToVisitPrescripted.push_back(next);
			} else {
				segmentsToVisitNotForbidden.push_back(next);
			}
		}
		next = next->next;
	}

	// Order is not important. Only to keep previous order.
	SHARED_PTR<RouteSegment> res = NULL;
	for (int i = segmentsToVisitNotForbidden.size()-1; i >= 0; --i)
	{
		segmentsToVisitNotForbidden[i]->next = res;
		res = segmentsToVisitNotForbidden[i];
	}
	for (int i = segmentsToVisitPrescripted.size()-1; i >= 0; --i)
	{
		segmentsToVisitPrescripted[i]->next = res;
		res = segmentsToVisitPrescripted[i];
	}
	return res;
}

bool checkSolution(RoutingContext* ctx,
		SHARED_PTR<RouteSegment> const & segment, int segmentEnd, SHARED_PTR<RouteSegment> const & next,
		VISITED_MAP const & oppositeSegments, bool reverseWay)
{
	// 1. Check if opposite segment found so we can stop calculations
	int64_t nts = (next->road->id << ROUTE_POINTS) + next->segmentStart;
	VISITED_MAP::const_iterator oS = oppositeSegments.find(nts);
	if (oS != oppositeSegments.end()) {
		// restrictions checked
		SHARED_PTR<RouteSegment> const & opposite = oS->second;
		if (opposite != NULL)
		{
			SHARED_PTR<FinalRouteSegment> frs = SHARED_PTR<FinalRouteSegment>(new FinalRouteSegment);
			frs->direct = segment;
			frs->reverseWaySearch = reverseWay;
			SHARED_PTR<RouteSegment> op = SHARED_PTR<RouteSegment>(new RouteSegment(segment->road, segmentEnd));
			op->parentRoute = opposite;
			op->parentSegmentEnd = next->getSegmentStart();
			frs->opposite = op;
			frs->distanceFromStart = opposite->distanceFromStart + segment->distanceFromStart;
			ctx->finalRouteSegment = frs;
			return true;
		}
	}
	return false;
}

bool processIntersections(RoutingContext* ctx, SEGMENTS_QUEUE& graphSegments, VISITED_MAP const & visitedSegments,
		VISITED_MAP const & oppositeSegments, double distFromStart, double distToFinalPoint,
		SHARED_PTR<RouteSegment> const & segment, int segmentEnd, SHARED_PTR<RouteSegment> const & inputNext,
		bool reverseWay) {
	// Calculate possible ways to put into priority queue
	SHARED_PTR<RouteSegment> next = inputNext;
	while (next != NULL)
	{
		int64_t nts = (next->road->id << ROUTE_POINTS) + next->segmentStart;  // TODO refactor
		if (checkSolution(ctx, segment, segmentEnd, next,
				oppositeSegments, reverseWay)) return true;

		if (visitedSegments.count(nts) == 0) {
			double distanceToEnd = h(ctx, distToFinalPoint, next);
			if (next->parentRoute == NULL
					|| roadPriorityComparator(next->distanceFromStart, next->distanceToEnd, distFromStart, distanceToEnd,
							ctx->getHeuristicCoefficient()) > 0) {
				if (next->parentRoute != NULL) {
					// already in queue remove it (we can not remove it)
OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Nuevo next.parent %d -> %d", next->parentRoute->road->id, segment->road->id);///
					next = SHARED_PTR<RouteSegment>(new RouteSegment(next->road, next->segmentStart));
				}
				next->distanceFromStart = distFromStart;
				next->distanceToEnd = distanceToEnd;
				// put additional information to recover whole route after
				next->parentRoute = segment;
				next->parentSegmentEnd = segmentEnd;
				if (TRACE_ROUTING)
				{
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
							" >>  >> next (%d{%d}, %d) name=%s dist=%f",
							next->road->id, next->road->pointsX.size(), next->segmentStart,
							next->road->getName().c_str(), next->distanceFromStart);
				}
				graphSegments.push(next);
			}
		} else {
			// the segment was already visited! We need to follow better route if it exists
			// that is very strange situation and almost exception (it can happen when we underestimate distnceToEnd)
			if (distFromStart < next->distanceFromStart && next->road->id != segment->road->id) {
				next->distanceFromStart = distFromStart;
				next->parentRoute = segment;
				next->parentSegmentEnd = segmentEnd;
			}
		}

		// iterate to next road
		next = next->next;
	}
	return false;
}

void RoutingContext::reregisterRouteDataObject(SHARED_PTR<RouteDataObject> o, int segmentStart, uint32_t x31, 
		uint32_t y31) {
	uint32_t z  = config.zoomToLoad;
	uint32_t xloc = (x31) >> (31 -z);
	uint32_t yloc = (y31) >> (31 -z);
	int64_t tileId = (xloc << z) + yloc;
	std::vector<SHARED_PTR<RoutingSubregionTile> >& subregions = indexedSubregions[tileId];
	for(uint j = 0; j < subregions.size(); j++) {
		if(subregions[j]->isLoaded()) {
			UNORDERED(map)<int64_t, SHARED_PTR<RouteSegment> >::iterator s = subregions[j]->routes.begin();
			while(s != subregions[j]->routes.end()) {
				SHARED_PTR<RouteSegment> seg = s->second;
				while(seg.get() != NULL) {
					if(seg->road->id == o->id  && seg->segmentStart > segmentStart) {
						seg->segmentStart ++;
					}
					seg = seg->next;
				}
				s++;
			}
		}
	}
}

SHARED_PTR<RouteSegment> findRouteSegment(int px, int py, RoutingContext* ctx) {
	std::vector<SHARED_PTR<RouteDataObject> > dataObjects;
	ctx->loadTileData(px, py, 17, dataObjects);
	if (dataObjects.empty()) {
		ctx->loadTileData(px, py, 15, dataObjects);
	}

	// Candidate
	RouteDataObject * road = NULL;
	size_t index = 0;
	int candidateX = -1;
	int candidateY = -1;
	double sdist = 0;
	std::vector<SHARED_PTR<RouteDataObject> >::const_iterator it = dataObjects.begin();
	for (; it!= dataObjects.end(); it++) {
		RouteDataObject * r = (*it).get();
		for (size_t j = 1; j < r->pointsX.size(); ++j) {
			// (px, py) projection over (j-1)(j) segment
			std::pair<int, int> pr = calculateProjectionPoint31(r->pointsX[j-1], r->pointsY[j-1],
					r->pointsX[j], r->pointsY[j],
					px, py);
			// Both distance and squared distance (we use) are monotone and positive functions.
			double currentsDist = squareDist31TileMetric(pr.first, pr.second, px, py);
			if (currentsDist < sdist || road == NULL) {
				// New candidate
				road = r;
				index = j;
				candidateX = pr.first;
				candidateY = pr.second;
				sdist = currentsDist;
			}
		}
	}
	if (road != NULL) {
		/*** Always add a new point to road to allow 'RoutePlannerFrontEnd.searchRoute' postprocessing.
		if ((candidateX == road->pointsX[index-1]) && (candidateY == road->pointsY[index-1]))
		{
			// Projection has same coordinates. None new.
			return (SHARED_PTR<RouteSegment>) new RouteSegment(road, index-1);
		}
		if ((candidateX == road->pointsX[index]) && (candidateY == road->pointsY[index]))
		{
			// Projection has same coordinates. None new.
			return (SHARED_PTR<RouteSegment>) new RouteSegment(road, index);
		}
		***/

		SHARED_PTR<RouteDataObject> proj = SHARED_PTR<RouteDataObject>(new RouteDataObject(*road));
		proj->pointsX.insert(proj->pointsX.begin() + index, candidateX);
		proj->pointsY.insert(proj->pointsY.begin() + index, candidateY);
		if (proj->pointTypes.size() > index) {
			proj->pointTypes.insert(proj->pointTypes.begin() + index, std::vector<uint32_t>());
		}
		// re-register the best road because one more point was inserted
		ctx->registerRouteDataObject(proj);
		return SHARED_PTR<RouteSegment>(new RouteSegment(proj, index));
	}
	return NULL;
}

bool combineTwoSegmentResult(RouteSegmentResult const & toAdd, RouteSegmentResult& previous) {
	bool ld = previous.endPointIndex > previous.startPointIndex;
	bool rd = toAdd.endPointIndex > toAdd.startPointIndex;
	if (rd == ld) {
		if (toAdd.startPointIndex == previous.endPointIndex) {
			previous.endPointIndex = toAdd.endPointIndex;
			previous.routingTime = previous.routingTime + toAdd.routingTime;
			return true;
		} else if (toAdd.endPointIndex == previous.startPointIndex) {
			previous.startPointIndex = toAdd.startPointIndex;
			previous.routingTime = previous.routingTime + toAdd.routingTime;
			return true;
		}
	}
	return false;
}

void addRouteSegmentToResult(std::vector<RouteSegmentResult>& result, RouteSegmentResult const & res) {
	if (res.endPointIndex != res.startPointIndex) {
		if (!result.empty()) {
			RouteSegmentResult & last = result[result.size() - 1];
			if (last.object->id == res.object->id) {
				if (combineTwoSegmentResult(res, last)) {
					return;
				}
			}
		}
		result.push_back(res);
	}
}

void attachConnectedRoads(RoutingContext* ctx, std::vector<RouteSegmentResult>& res) {
	std::vector<RouteSegmentResult>::iterator it = res.begin();
	for (; it != res.end(); it++) {
		bool plus = it->startPointIndex < it->endPointIndex;
		int j = it->startPointIndex;
		do {
			SHARED_PTR<RouteSegment> s = ctx->loadRouteSegment(it->object->pointsX[j], it->object->pointsY[j]);
			std::vector<RouteSegmentResult> r;
			RouteSegment* rs = s.get();
			while(rs != NULL) {
				RouteSegmentResult res(rs->road, rs->getSegmentStart(), rs->getSegmentStart());
				r.push_back(res);
				rs = rs->next.get();
			}
			it->attachedRoutes.push_back(r);
			j = plus ? j + 1 : j - 1;
		}while(j != it->endPointIndex);
	}
}

///
float calcRoutingTime(float parentRoutingTime, SHARED_PTR<RouteSegment> finalSegment, 
	SHARED_PTR<RouteSegment> segment, RouteSegmentResult& res) {
	if(segment.get() != finalSegment.get()) {
		if(parentRoutingTime != -1) {
			res.routingTime = parentRoutingTime - segment->distanceFromStart;
		}
		parentRoutingTime = segment->distanceFromStart;
	}
	return parentRoutingTime;

}
std::vector<RouteSegmentResult> convertFinalSegmentToResults(RoutingContext* ctx) {
	std::vector<RouteSegmentResult> result;
	if (ctx->finalRouteSegment != NULL) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Routing calculated time distance %f", ctx->finalRouteSegment->distanceFromStart);
		FinalRouteSegment * const finalSegment = ctx->finalRouteSegment.get();

		// Get results from direct direction roads
		SHARED_PTR<RouteSegment> segment = finalSegment->reverseWaySearch ? finalSegment->opposite->parentRoute : finalSegment->direct;
		int parentSegmentEnd =
				finalSegment->reverseWaySearch ?
						finalSegment->opposite->parentSegmentEnd : finalSegment->opposite->getSegmentStart();
		while (segment != NULL) {
			RouteSegmentResult res(segment->road, segment->getSegmentStart(), parentSegmentEnd);
			///parentRoutingTime = calcRoutingTime(parentRoutingTime, finalSegment, segment, res);
			parentSegmentEnd = segment->parentSegmentEnd;
			segment = segment->parentRoute;
			addRouteSegmentToResult(result, res);
		}
		std::reverse(result.begin(), result.end());

		// Get results from opposite direction roads
		segment = finalSegment->reverseWaySearch ? finalSegment->direct : finalSegment->opposite->parentRoute;
		int parentSegmentStart =
				finalSegment->reverseWaySearch ?
						finalSegment->opposite->getSegmentStart() : finalSegment->opposite->parentSegmentEnd;
		while (segment != NULL) {
			RouteSegmentResult res(segment->road, parentSegmentStart, segment->getSegmentStart());
			parentSegmentStart = segment->parentSegmentEnd;
			segment = segment->parentRoute;
			addRouteSegmentToResult(result, res);
		}
	}

	return result;
}

std::vector<RouteSegmentResult> searchRouteInternal(RoutingContext* ctx, bool leftSideNavigation) {
	SHARED_PTR<RouteSegment> start = findRouteSegment(ctx->startX, ctx->startY, ctx);
	if (start == NULL) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Start point was not found [Native]");
		if (ctx->progress != NULL) {
			ctx->progress->setSegmentNotFound(0);
		}
		return std::vector<RouteSegmentResult>();
	} else {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Start point was found %lld [Native]", start->road->id);
	}
	SHARED_PTR<RouteSegment> end = findRouteSegment(ctx->targetX, ctx->targetY, ctx);
	if (end == NULL) {
		if(ctx->progress != NULL) {
			ctx->progress->setSegmentNotFound(1);
		}
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "End point was not found [Native]");
		return std::vector<RouteSegmentResult>();
	} else {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "End point was found %lld [Native]", end->road->id);
	}

	// Postprocessing to manager start and end over the same road segment.
	if (start->road->id == end->road->id)
	{
		// same road id, end is a better copy.
		start->road = end->road;
		if (start->segmentStart >= end->segmentStart)
		{
			start->segmentStart++;
		}
	}

	searchRouteInternal(ctx, start, end, leftSideNavigation);
	std::vector<RouteSegmentResult> res = convertFinalSegmentToResults(ctx);
	attachConnectedRoads(ctx, res);
	return res;
}

bool compareRoutingSubregionTile(SHARED_PTR<RoutingSubregionTile> const & o1, SHARED_PTR<RoutingSubregionTile> const & o2) {
	int v1 = (o1->access + 1) * pow(10, o1->getUnloadCount()-1);
	int v2 = (o2->access + 1) * pow(10, o2->getUnloadCount()-1);
	return v1 < v2;
}
