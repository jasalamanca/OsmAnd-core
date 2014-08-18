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

void printRoad(const char* prefix, SHARED_PTR<RouteSegment> const & segment) {
       OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "%s Road id=%lld ind=%d ds=%f es=%f pend=%d parent=%lld",
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

static double h(RoutingContext* ctx, int targetEndX, int targetEndY, int startX, int startY) {
	double distance = distance31TileMetric(startX, startY, targetEndX, targetEndY);
	return distance / ctx->config.router.getMaxDefaultSpeed();
}

struct SegmentsComparator
		: public std::binary_function<SHARED_PTR<RouteSegment>, SHARED_PTR<RouteSegment>, bool>
{
public:
	SegmentsComparator()
	{}
	inline bool operator()(const SHARED_PTR<RouteSegment> & n1, const SHARED_PTR<RouteSegment> & n2) const
	{
		// f(x) = g(x) + h(x)  --- g(x) - distanceFromStart, h(x) - distanceToEnd (a guess)
		// We want the shortest cost to be choosen fron priority_queue, so we need to use > operator.
		float f1 = n1->distanceFromStart + n1->distanceToEnd;
		float f2 = n2->distanceFromStart + n2->distanceToEnd;
		if (f1 == f2) {
			return n1->distanceFromStart > n2->distanceFromStart;
		}
		return f1 > f2;
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
		if (checkSolution(ctx, segment, segmentEnd, next,
				oppositeSegments, reverseWay)) return true;

		int64_t nts = (next->road->id << ROUTE_POINTS) + next->segmentStart;  // TODO refactor
		if (visitedSegments.count(nts) == 0) {
			if (next->parentRoute == NULL
					|| next->distanceFromStart > distFromStart) {
//					|| roadPriorityComparator(next->distanceFromStart, next->distanceToEnd, distFromStart, distToFinalPoint,
//							ctx->getHeuristicCoefficient()) > 0) {
				if (next->parentRoute != NULL) {
					// already in queue remove it (we can not remove it)
OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Nuevo next.parent %d -> %d", next->parentRoute->road->id, segment->road->id);///
					next = SHARED_PTR<RouteSegment>(new RouteSegment(next->road, next->segmentStart));
				}
				next->distanceFromStart = distFromStart;
				next->distanceToEnd = distToFinalPoint;
				// put additional information to recover whole route after
				next->parentRoute = segment;
				next->parentSegmentEnd = segmentEnd;
				if (TRACE_ROUTING)
				{
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
							" >>  >> next (%d{%d}, %d) name=%s"
							"\n\t\tf{g+h}=%f{%f+%f}",
							next->road->id, next->road->pointsX.size(), next->segmentStart,
							next->road->getName().c_str(),
							next->distanceFromStart+next->distanceToEnd, next->distanceFromStart, next->distanceToEnd);
				}
				graphSegments.push(next);
			}
		} else {
			if (distFromStart < next->distanceFromStart && next->road->id != segment->road->id) {
			// the segment was already visited! We need to follow better route if it exists
			// that is very strange situation and almost exception (it can happen when we underestimate distnceToEnd)
OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "YA visitado next (%d, %d) segment (%d, %d)",
		next->road->id, next->segmentStart, segment->road->id, segment->segmentStart);///
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
//if (TRACE_ROUTING) std::cerr << "Restrictions TEST " << next->road->id << '[' << next->segmentStart << ']' << std::endl;
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
if (TRACE_ROUTING) std::cerr << "Restrictions NOT FORBIDDEN " << res->road->id << '[' << res->segmentStart << ']' << std::endl;
	}
	for (int i = segmentsToVisitPrescripted.size()-1; i >= 0; --i)
	{
		segmentsToVisitPrescripted[i]->next = res;
		res = segmentsToVisitPrescripted[i];
if (TRACE_ROUTING) std::cerr << "Restrictions PRESCRIPTED " << res->road->id << '[' << res->segmentStart << ']' << std::endl;
	}
	return res;
}

bool visitRouteSegment(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE & graphSegments,
		VISITED_MAP & visitedSegments, int targetEndX, int targetEndY,
		SHARED_PTR<RouteSegment> const & segment,
		VISITED_MAP const & oppositeSegments, int delta, double obstacleTime)
{
	SHARED_PTR<RouteDataObject> const & road = segment->road;
	int start = segment->segmentStart + delta;
	int end = (delta == 1)?road->pointsX.size():-1;
	double distOnRoadToPass = 0;
	while (start != end)
	{
		// algorithm should visit all reacheable points on the road
		int64_t nts = (road->id << ROUTE_POINTS) + start;
		if (visitedSegments.count(nts) != 0)
		{
			start += delta;
			continue;
		}
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
		if (next != NULL)
		{
			// Using A* routing algorithm
			// g(x) - calculate distance to that point and calculate time
			double priority = ctx->config.router.defineSpeedPriority(road);
			double speed = ctx->config.router.defineRoutingSpeed(road) * priority;
			if (speed == 0) {
				speed = ctx->config.router.getMinDefaultSpeed() * priority;
			}
			double distStartObstacles = segment->distanceFromStart + obstacleTime + distOnRoadToPass / speed;
			//double distToFinalPoint = distance31TileMetric(x, y, targetEndX, targetEndY);
			//double distanceToEnd = h(ctx, distToFinalPoint, next);
			double distToFinalPoint = h(ctx, x, y, targetEndX, targetEndY);

			if (TRACE_ROUTING)
			{
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
						">> ?%d dist=%f time=%f",
						start, distOnRoadToPass, distOnRoadToPass/speed);
			}

			if (processIntersections(ctx, graphSegments, visitedSegments, oppositeSegments,
					distStartObstacles, distToFinalPoint, segment, start, next, reverseWaySearch) ) return true;
		}  // end of next != NULL
	// next
		start += delta;
	}
	return false;
}

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
				"Process segment (%d{%d}, %d) name=%s"
				"\n\t\tspeed=%f prio=%f roadDirection=%d"
				"\n\t\tf{g+h}=%f{%f+%f}",
				road->id, road->pointsX.size(), start, road->getName().c_str(),
				ctx->config.router.defineRoutingSpeed(road), ctx->config.router.defineSpeedPriority(road), roadDirection,
				segment->distanceFromStart+segment->distanceToEnd, segment->distanceFromStart, segment->distanceToEnd);
	}

	ctx->loadRoad(road);

	if ( ((!reverseWaySearch && roadDirection >= 0)	|| (reverseWaySearch && roadDirection <= 0))
			&& start < (road->pointsX.size()-1) )
	{
		// We have bigger indexes to visit.
		// We penalize if trying the reverse direction.
		double obstacleTime = 0;////(ctx->firstRoadId == nt && ctx->firstRoadDirection < 0)?500:0;
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
		double obstacleTime = 0;////(ctx->firstRoadId == nt && ctx->firstRoadDirection > 0)?500:0;
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

void searchRouteInternal(RoutingContext* ctx,
		SHARED_PTR<RouteSegment> const & start, SHARED_PTR<RouteSegment> const & end,
		bool leftSideNavigation) {
	// FIXME intermediate points
	// measure time
	ctx->visitedSegments = 0;
	int iterationsToUpdate = 0;
	ctx->timeToCalculate.Start();
/***
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
***/
/***
	auto cmp = [](SHARED_PTR<RouteSegment> const & rs1, SHARED_PTR<RouteSegment> const & rs2)
			{
		// f(x) = g(x) + h(x)  --- g(x) = distanceFromStart, h(x) = distanceToEnd
		// Return f(rs1) < f(rs2)
		return (rs1->distanceFromStart + rs1->distanceToEnd) < (rs1->distanceFromStart + rs1->distanceToEnd);
			};
***/

	SegmentsComparator sgmCmp;
	SEGMENTS_QUEUE graphDirectSegments(sgmCmp);
	SEGMENTS_QUEUE graphReverseSegments(sgmCmp);

	// Set to not visit one segment twice (stores road.id << X + segmentStart)
	VISITED_MAP visitedDirectSegments;
	VISITED_MAP visitedReverseSegments;

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

	// Search from end or from start
	bool inverse = false;
	SEGMENTS_QUEUE * graphSegments = inverse?&graphReverseSegments:&graphDirectSegments;

	while (!graphSegments->empty())
	{
		SHARED_PTR<RouteSegment> segment = graphSegments->top();
		graphSegments->pop();
		if (!inverse) {
			if (processRouteSegment(ctx, false, graphDirectSegments, visitedDirectSegments,
					targetEndX, targetEndY, segment,
					visitedReverseSegments))
				break;
		} else {
			if (processRouteSegment(ctx, true, graphReverseSegments, visitedReverseSegments,
					startX, startY, segment,
					visitedDirectSegments))
				break;
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

		if (graphDirectSegments.empty())
		{
			if (graphReverseSegments.empty())
				break; // NO route found
			else
				inverse = true; // Only reverse openset
		}
		else
		{
			if (graphReverseSegments.empty())
				inverse = false;  // Only direct openset
			else
				inverse = !inverse;  // Change direction
				//inverse = graphReverseSegments.top()->f() < graphDirectSegments.top()->f(); // Better on reverse
		}
		graphSegments = inverse?&graphReverseSegments:&graphDirectSegments;

		// check if interrupted
		if(ctx->isInterrupted()) {
			return;
		}
	}
	ctx->timeToCalculate.Pause();
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "[Native] Result visited (visited roads %d, visited segments %d / %d , queue sizes %d / %d ) ",
			ctx-> visitedSegments, visitedDirectSegments.size(), visitedReverseSegments.size(),
			graphDirectSegments.size(),graphReverseSegments.size());
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "[Native] Result timing (time to load %d, time to calc %d, loaded tiles %d) ",
			ctx->timeToLoad.GetElapsedMs(), ctx->timeToCalculate.GetElapsedMs(), ctx->loadedTiles);
	int sz = calculateSizeOfSearchMaps(graphDirectSegments, graphReverseSegments, visitedDirectSegments, visitedReverseSegments);
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "[Native] Memory occupied (Routing context %d Kb, search %d Kb)", ctx->getSize()/1024, sz/1024);
}

bool _checkSolution(RoutingContext* ctx,
		SHARED_PTR<RouteSegment> const & segment, int segmentEnd, int endX, int endY)
{
	if (segment->road->pointsX[segmentEnd] == endX
			&& segment->road->pointsY[segmentEnd] == endY)
	{
			SHARED_PTR<FinalRouteSegment> frs = SHARED_PTR<FinalRouteSegment>(new FinalRouteSegment);
			frs->direct = segment;
			frs->reverseWaySearch = false;
			SHARED_PTR<RouteSegment> op = SHARED_PTR<RouteSegment>(new RouteSegment(segment->road, segmentEnd));
			frs->opposite = op;
			frs->distanceFromStart = segment->distanceFromStart;

			ctx->finalRouteSegment = frs;
			return true;
	}
	return false;
}

bool _checkSolution(RoutingContext* ctx,
		SHARED_PTR<RouteSegment> const & segment, SHARED_PTR<RouteSegment> const & end)
{
	// If end reached we are the champions
	if (segment->road->id == end->road->id
		&& segment->segmentStart == end->segmentStart)
	{
		SHARED_PTR<FinalRouteSegment> frs = SHARED_PTR<FinalRouteSegment>(new FinalRouteSegment);
		frs->direct = segment;
		frs->reverseWaySearch = false;
		frs->opposite = end;
		frs->distanceFromStart = segment->distanceFromStart;

		ctx->finalRouteSegment = frs;
		return true;
	}
	return false;
}

void _processIntersections(RoutingContext* ctx, SEGMENTS_QUEUE& graphSegments, VISITED_MAP const & visitedSegments,
		double distFromStart, double distToFinalPoint,
		SHARED_PTR<RouteSegment> const & segment, int segmentEnd, SHARED_PTR<RouteSegment> next)
{
	// For each neighbor
	while (next != NULL)
	{
		int64_t nts = (next->road->id << ROUTE_POINTS) + next->segmentStart;  // TODO refactor
		if (visitedSegments.count(nts) == 0)
		{ // NOT in closed set
			if (next->parentRoute == NULL // NOT in open set (If h(x) is monotone then will always occur)
				|| next->distanceFromStart > distFromStart) // Thru segment we have a less costly path
			{
				if (next->parentRoute != NULL)
				{
					// already in queue remove it (we can not remove it)
OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Nuevo next.parent %d -> %d", next->parentRoute->road->id, segment->road->id);///
					next = SHARED_PTR<RouteSegment>(new RouteSegment(next->road, next->segmentStart));
				}
				next->distanceFromStart = distFromStart;
				next->distanceToEnd = distToFinalPoint;
				// put additional information to recover whole route after
				next->parentRoute = segment;
				next->parentSegmentEnd = segmentEnd;
				if (TRACE_ROUTING)
				{
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
							" >>  >> next (%d{%d}, %d) name=%s"
							"\n\t\tf{g+h}=%f{%f+%f}",
							next->road->id, next->road->pointsX.size(), next->segmentStart,
							next->road->getName().c_str(),
							next->distanceFromStart+next->distanceToEnd, next->distanceFromStart, next->distanceToEnd);
				}
				// Add to open set
				graphSegments.push(next);
			}
		}
		// iterate to next
		next = next->next;
	}
}

SHARED_PTR<RouteSegment> _proccessRestrictions(RoutingContext* ctx, SHARED_PTR<RouteDataObject> const & road,
		SHARED_PTR<RouteSegment> const & inputNext)
{
	if (road->restrictions.empty()) {
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
if (TRACE_ROUTING) std::cerr << "Restrictions TEST " << next->road->id << '[' << next->segmentStart << ']' << std::endl;
		int type = -1;
		int64_t id = next->road->id;
		for (size_t i = 0; i < road->restrictions.size(); i++) {
			if ((road->restrictions[i] >> 3) == id) {
				type = road->restrictions[i] & 7;
				break;
			}
		}
		if (type != -1 || exclusiveRestriction)
		{
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "type=%d exclusive=%d %d->%d",
					type, exclusiveRestriction, road->id, next->road->id);
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
			exclusiveRestriction = true;
			segmentsToVisitNotForbidden.clear();
			segmentsToVisitPrescripted.push_back(next);
		}
		next = next->next;
	}

	// Order is not important. Only to keep previous order.
	SHARED_PTR<RouteSegment> res = NULL;
	for (int i = segmentsToVisitNotForbidden.size()-1; i >= 0; --i)
	{
		segmentsToVisitNotForbidden[i]->next = res;
		res = segmentsToVisitNotForbidden[i];
if (TRACE_ROUTING) std::cerr << "Restrictions NOT FORBIDDEN " << res->road->id << '[' << res->segmentStart << ']' << std::endl;
	}
	for (int i = segmentsToVisitPrescripted.size()-1; i >= 0; --i)
	{
		segmentsToVisitPrescripted[i]->next = res;
		res = segmentsToVisitPrescripted[i];
if (TRACE_ROUTING) std::cerr << "Restrictions PRESCRIPTED " << res->road->id << '[' << res->segmentStart << ']' << std::endl;
	}
	return res;
}

bool _visitRouteSegment(RoutingContext* ctx, SEGMENTS_QUEUE & graphSegments,	VISITED_MAP & visitedSegments,
		int targetEndX, int targetEndY,
		SHARED_PTR<RouteSegment> const & segment, int delta,
		double obstacleTime)
{
	SHARED_PTR<RouteDataObject> const & road = segment->road;
	int start = segment->segmentStart + delta;
	int end = (delta == 1)?road->pointsX.size():-1;
	double distOnRoadToPass = 0;
	// algorithm should visit all reacheable points on the road
	while (start != end)
	{
		int64_t nts = (road->id << ROUTE_POINTS) + start;
		// Uninteresting RouteSegment neighbor goes directly to closed set
		if (visitedSegments.count(nts) != 0)
		{
			start += delta;
			continue;
		}
		visitedSegments[nts] = NULL;

		// and we check for end
		if (_checkSolution(ctx, segment, start, targetEndX, targetEndY)) return true;

		// 2. calculate point and try to load neighbor ways if they are not loaded
		int x = road->pointsX[start];
		int y = road->pointsY[start];
		distOnRoadToPass += distance31TileMetric(x, y,
				road->pointsX[start-delta], road->pointsY[start-delta]);

		// 2.1 check possible obstacle plus time
		double obstacle = ctx->config.router.defineRoutingObstacle(road, start);
		if (obstacle < 0)
		{
			continue;
		}
		obstacle += obstacleTime;
		// all the possible intersections. A* nodes can have many different RouteSegments representing it
		SHARED_PTR<RouteSegment> next = ctx->loadRouteSegment(x, y);
		// Be aware of restrictions
		next = _proccessRestrictions(ctx, segment->road, next);
		if (next != NULL) {
			// Using A* routing algorithm
			// g(x) - calculate distance to that point and calculate time
			double priority = ctx->config.router.defineSpeedPriority(road);
			double speed = ctx->config.router.defineRoutingSpeed(road) * priority;
			if (speed == 0) {
				speed = ctx->config.router.getMinDefaultSpeed() * priority;
			}
			double distStartObstacles = segment->distanceFromStart + obstacle + distOnRoadToPass / speed;
			double distToFinalPoint = h(ctx, x, y, targetEndX, targetEndY);

			if (TRACE_ROUTING)
			{
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
						">> ?%d dist=%f time=%f",
						start, distOnRoadToPass, distOnRoadToPass/speed);
			}

			// For each interesting neighbor
			_processIntersections(ctx, graphSegments, visitedSegments,
					distStartObstacles, distToFinalPoint, segment, start, next);
		}  // end of next != NULL
	// next
		start += delta;
	}
	return false;
}

bool _processRouteSegment(RoutingContext* ctx, SEGMENTS_QUEUE& graphSegments,
		VISITED_MAP& visitedSegments, int targetEndX, int targetEndY, SHARED_PTR<RouteSegment> const & segment)
{
	SHARED_PTR<RouteDataObject> const & road = segment->road;
	int start = segment->segmentStart;
	int roadDirection = ctx->config.router.isOneWay(road);
	if (TRACE_ROUTING)
	{
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
				"Process segment (%d{%d}, %d) name=%s"
				"\n\t\tspeed=%f prio=%f roadDirection=%d"
				"\n\t\tf{g+h}=%f{%f+%f}",
				road->id, road->pointsX.size(), start, road->getName().c_str(),
				ctx->config.router.defineRoutingSpeed(road), ctx->config.router.defineSpeedPriority(road), roadDirection,
				segment->distanceFromStart+segment->distanceToEnd, segment->distanceFromStart, segment->distanceToEnd);
	}

	// To reduce reading from files
	ctx->loadRoad(road);

	if ( (roadDirection >= 0) && start < (road->pointsX.size()-1) )
	{
		// We have bigger indexes to visit.
		double obstacleTime = 0;
		if (segment->parentRoute != NULL)
		{
			obstacleTime = ctx->config.router.calculateTurnTime(segment, road->pointsX.size()-1,
					segment->parentRoute, segment->parentSegmentEnd);
		}
		if (_visitRouteSegment(ctx, graphSegments, visitedSegments, targetEndX, targetEndY,	segment, 1, obstacleTime))
			return true;
	}
	if ( (roadDirection <= 0) && start > 0 )
	{
		// We have smaller indexes to visit
		double obstacleTime = 0;
		if (segment->parentRoute != NULL)
		{
			obstacleTime = ctx->config.router.calculateTurnTime(segment, 0,
					segment->parentRoute, segment->parentSegmentEnd);
		}
		if (_visitRouteSegment(ctx, graphSegments, visitedSegments, targetEndX, targetEndY,	segment, -1, obstacleTime))
			return true;
	}
	return false;
}

void _searchRouteInternal(RoutingContext* ctx,
		SHARED_PTR<RouteSegment> const & start, SHARED_PTR<RouteSegment> const & end)
{
	// FIXME intermediate points
	// measure time
	ctx->visitedSegments = 0;
	int iterationsToUpdate = 0;
	ctx->timeToCalculate.Start();

	SegmentsComparator sgmCmp;
	SEGMENTS_QUEUE graphSegments(sgmCmp);

	// Set to not visit one segment twice (stores road.id << X + segmentStart)
	// Closedset empty
	VISITED_MAP visitedSegments;

	// for start : f(start) = g(start) + h(start) = 0 + h(start) = h(start)
	int targetEndX = end->road->pointsX[end->segmentStart];
	int targetEndY = end->road->pointsY[end->segmentStart];
	int startX = start->road->pointsX[start->segmentStart];
	int startY = start->road->pointsY[start->segmentStart];
	// f(start) = g(start) + h(start) = 0 + h(start)   g(x) -> x.distanceFromStart h(x) -> x.distanceToEnd
	start->distanceToEnd = h(ctx, targetEndX, targetEndY, startX, startY);
	// add start to openset
	graphSegments.push(start);
	// Path from is empty. path(x) -> x.parentRoute

	while (!graphSegments.empty())
	{
		// Select node with minimal f(x)
		SHARED_PTR<RouteSegment> segment = graphSegments.top();

		if (_checkSolution(ctx, segment, end)) break; // Solution found

		// Remove from openset
		graphSegments.pop();
		// Add to closed
		int64_t nt = (segment->road->id << ROUTE_POINTS) + segment->segmentStart;
		visitedSegments[nt] = segment;
		ctx->visitedSegments++;

		if (_processRouteSegment(ctx, graphSegments, visitedSegments,
				targetEndX, targetEndY,	segment))
			break;

		if (ctx->progress != NULL && iterationsToUpdate-- < 0) {
			iterationsToUpdate = 100;
			ctx->progress->updateStatus(graphSegments.empty()? 0 :graphSegments.top()->distanceFromStart,
					graphSegments.size(),
					0,
					0);
			if(ctx->progress->isCancelled()) break;
		}
		// check if interrupted
		if(ctx->isInterrupted()) return;
	}
	ctx->timeToCalculate.Pause();
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "[Native] Result visited (visited roads %d, visited segments %d / %d , queue sizes %d / %d ) ",
			ctx->visitedSegments, visitedSegments.size(), 0,
			graphSegments.size(),0);
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "[Native] Result timing (time to load %d, time to calc %d, loaded tiles %d) ",
			ctx->timeToLoad.GetElapsedMs(), ctx->timeToCalculate.GetElapsedMs(), ctx->loadedTiles);
	int sz = calculateSizeOfSearchMaps(graphSegments, SEGMENTS_QUEUE(sgmCmp), visitedSegments, VISITED_MAP());
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "[Native] Memory occupied (Routing context %d Kb, search %d Kb)", ctx->getSize()/1024, sz/1024);
}

void RoutingQuery(bbox_t & b, RouteDataObjects_t & output);
SHARED_PTR<RouteSegment> findRouteSegment(int px, int py, RoutingContext* ctx)
{
//	OsmAnd::ElapsedTimer timer;
//	timer.Start();
	bbox_t b(point_t(px,py), point_t(px,py));
	RouteDataObjects_t dataObjects;
	RoutingQuery(b, dataObjects);

	// Candidate
	RouteDataObject const * road = NULL;
	size_t index = 0;
	int candidateX = -1;
	int candidateY = -1;
	double sdist = 0;
	RouteDataObjects_t::const_iterator it = dataObjects.begin();
	for (; it!= dataObjects.end(); it++) {
		RouteDataObject * r = *it;
		if (r == NULL or !ctx->acceptLine(SHARED_PTR<RouteDataObject>(new RouteDataObject(*r))))////
		{
if (TRACE_ROUTING)
{
std::cerr << "FindRS NAT ";
	if (r==NULL)
		std::cerr << "NULL";
	else
		std::cerr << r->id << " NOT accepted";
std::cerr << std::endl;
}
			continue;
		}
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
//std::cerr << "FindRS candidate " << road->id << "[" << index << "]=(" << candidateX << ',' << candidateY << ") dist=" << sdist << std::endl;
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
//		timer.Pause();
//		std::cerr << "NAT findRS time=" << timer.GetElapsedMs() << std::endl;
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
	for (; it != res.end(); it++)
	{
		int advance = (it->startPointIndex < it->endPointIndex)?1:-1;
		int j = it->startPointIndex;
		do {
			SHARED_PTR<RouteSegment> s = ctx->loadRouteSegment(it->object->pointsX[j], it->object->pointsY[j]);
			std::vector<RouteSegmentResult> r;
			while(s != NULL)
			{
				r.push_back(RouteSegmentResult(s->road, s->getSegmentStart(), s->getSegmentStart()));
				s = s->next;
			}
			it->attachedRoutes.push_back(std::move(r));
			j += advance;
		} while (j != it->endPointIndex);
	}
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

	// Postprocessing to manage start and end over the same road segment.
	if (start->road->id == end->road->id)
	{
		// same road id, end is a better copy.
		start->road = end->road;
		if (start->segmentStart >= end->segmentStart)
		{
			start->segmentStart++;
		}
	}

	// Bidirectional search
	searchRouteInternal(ctx, start, end, leftSideNavigation);
	// Unidirectional search
	//_searchRouteInternal(ctx, start, end);
	std::vector<RouteSegmentResult> res = convertFinalSegmentToResults(ctx);
	attachConnectedRoads(ctx, res);
	return res;
}
