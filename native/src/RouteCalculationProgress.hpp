/*
 * RouteCalculationProgress.hpp
 *
 *  Created on: 28/08/2014
 *      Author: javier
 */

#ifndef ROUTECALCULATIONPROGRESS_HPP_
#define ROUTECALCULATIONPROGRESS_HPP_

class RouteCalculationProgress
{
protected:
	int segmentNotFound;
	float distanceFromBegin;
	int directSegmentQueueSize;
	float distanceFromEnd;
	int reverseSegmentQueueSize;

	bool cancelled;
public:
	RouteCalculationProgress() : segmentNotFound(-1), distanceFromBegin(0),
		directSegmentQueueSize(0), distanceFromEnd(0), reverseSegmentQueueSize(0), cancelled(false){
	}

	virtual ~RouteCalculationProgress(){};

	virtual bool isCancelled() const {
		return cancelled;
	}

	virtual void setSegmentNotFound(int s){
		segmentNotFound = s;
	}

	virtual void updateStatus(float distanceFromBegin,	int directSegmentQueueSize,	float distanceFromEnd,
			int reverseSegmentQueueSize) {
		this->distanceFromBegin = std::max(distanceFromBegin, this->distanceFromBegin );
		this->distanceFromEnd = std::max(distanceFromEnd,this->distanceFromEnd);
		this->directSegmentQueueSize = directSegmentQueueSize;
		this->reverseSegmentQueueSize = reverseSegmentQueueSize;
	}
};

#endif /* ROUTECALCULATIONPROGRESS_HPP_ */
