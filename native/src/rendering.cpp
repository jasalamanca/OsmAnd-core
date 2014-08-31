#include <SkCanvas.h>
#include <SkColorFilter.h>
#include <SkBitmapProcShader.h>
#include <SkPathEffect.h>
#include <SkBlurDrawLooper.h>
#include <SkDashPathEffect.h>
#include <SkPaint.h>

#include "textdraw.cpp"
#include "mapObjects.h"
#include "rendering.h"
#include "Logging.h"
#include "Internal.h"

#if defined(WIN32)
#undef min
#endif

struct MapDataObjectPrimitive {
	MapDataObject* obj;
	int typeInd;
	int order;
	int objectType;
};

void calcPoint(std::pair<int, int> const & c, RenderingContext & rc)
{
    rc.pointCount++;

	double tx = c.first/ (rc.tileDivisor);
	double ty = c.second / (rc.tileDivisor);

    float dTileX = tx - rc.getLeft();
    float dTileY = ty - rc.getTop();
    rc.calcX = rc.cosRotateTileSize * dTileX - rc.sinRotateTileSize * dTileY;
    rc.calcY = rc.sinRotateTileSize * dTileX + rc.cosRotateTileSize * dTileY;

    if (rc.calcX >= 0 && rc.calcX < rc.getWidth()&& rc.calcY >= 0 && rc.calcY < rc.getHeight())
        rc.pointInsideCount++;
}

UNORDERED(map)<std::string, SkPathEffect*> pathEffects;
SkPathEffect* getDashEffect(RenderingContext const & rc, std::string const & input)
{
	// I think input comes from xml config files and then changing locales is necessary
	char * oldLocales = setlocale(LC_NUMERIC, NULL);
	setlocale(LC_NUMERIC, "C");

    const char* chars = input.c_str();
    int i = 0;
    char fval[10];
    int flength = 0;
    std::vector<float> primFloats;
    bool afterColon = false;
    std::string hash = "";
    for(;;i++) {
        if(chars[i] != '_' && chars[i] != 0 && chars[i] != ':') {
           // suppose it is a character
           fval[flength++] = chars[i];
        } else {
            fval[flength] = 0;
			float parsed = 0;
			if(flength > 0) {	
			   parsed = atof(fval);
			}

			if(afterColon) {
				primFloats[primFloats.size() - 1] += parsed;
			} else {
				parsed = rc.getDensityValue(parsed);
				primFloats.push_back(parsed); 
			}
			hash += (parsed * 10);
			flength = 0;
			
			if(chars[i] == ':') {
				afterColon = true;
			} else {
				afterColon = false;
			}
            if(chars[i] == 0) break;                
        }
    }

    // restoring locales
    setlocale(LC_NUMERIC, oldLocales);

    if(pathEffects.find(hash) != pathEffects.end())
        return pathEffects[hash];

    SkPathEffect* r = new SkDashPathEffect(&primFloats[0], primFloats.size(), 0);
    pathEffects[hash] = r;
    return r;
}

int updatePaint(RenderingRuleSearchRequest const * req, SkPaint & paint, int ind, int area, RenderingContext & rc)
{
    RenderingRuleProperty* rColor;
    RenderingRuleProperty* rStrokeW;
    RenderingRuleProperty* rCap;
    RenderingRuleProperty* rPathEff;
    if (ind == 0)
    {
        rColor = req->props()->R_COLOR;
        rStrokeW = req->props()->R_STROKE_WIDTH;
        rCap = req->props()->R_CAP;
        rPathEff = req->props()->R_PATH_EFFECT;
    }
    else if (ind == 1)
    {
        rColor = req->props()->R_COLOR_2;
        rStrokeW = req->props()->R_STROKE_WIDTH_2;
        rCap = req->props()->R_CAP_2;
        rPathEff = req->props()->R_PATH_EFFECT_2;
    }
    else if (ind == -1)
    {
        rColor = req->props()->R_COLOR_0;
        rStrokeW = req->props()->R_STROKE_WIDTH_0;
        rCap = req->props()->R_CAP_0;
        rPathEff = req->props()->R_PATH_EFFECT_0;
    }
    else if (ind == -2)
    {
        rColor = req->props()->R_COLOR__1;
        rStrokeW = req->props()->R_STROKE_WIDTH__1;
        rCap = req->props()->R_CAP__1;
        rPathEff = req->props()->R_PATH_EFFECT__1;
    }
    else if (ind == 2)
    {
        rColor = req->props()->R_COLOR_3;
        rStrokeW = req->props()->R_STROKE_WIDTH_3;
        rCap = req->props()->R_CAP_3;
        rPathEff = req->props()->R_PATH_EFFECT_3;
    }
    else if (ind == 3)
    {
        rColor = req->props()->R_COLOR_4;
        rStrokeW = req->props()->R_STROKE_WIDTH_4;
        rCap = req->props()->R_CAP_4;
        rPathEff = req->props()->R_PATH_EFFECT_4;
    } else 
    {
    	rColor = req->props()->R_COLOR_4;
        rStrokeW = req->props()->R_STROKE_WIDTH_4;
        rCap = req->props()->R_CAP_4;
        rPathEff = req->props()->R_PATH_EFFECT_4;
    }

    if (area)
    {
    	paint.setColorFilter(NULL);
    	paint.setShader(NULL);
    	paint.setLooper(NULL);
    	paint.setStyle(SkPaint::kStrokeAndFill_Style);
    	paint.setStrokeWidth(0);
    }
    else
    {
        float stroke = getDensityValue(rc, req, rStrokeW);
        if (!(stroke > 0))
            return 0;
        paint.setColorFilter(NULL);
        paint.setShader(NULL);
        paint.setLooper(NULL);

        paint.setStyle(SkPaint::kStroke_Style);
        paint.setStrokeWidth(stroke);
        std::string cap = req->getStringPropertyValue(rCap);
        std::string pathEff = req->getStringPropertyValue(rPathEff);

        if (cap == "BUTT" || cap == "")
            paint.setStrokeCap(SkPaint::kButt_Cap);
        else if (cap == "ROUND")
            paint.setStrokeCap(SkPaint::kRound_Cap);
        else if (cap == "SQUARE")
            paint.setStrokeCap(SkPaint::kSquare_Cap);
        else
            paint.setStrokeCap(SkPaint::kButt_Cap);

        if (pathEff.size() > 0)
        {
            SkPathEffect* p = getDashEffect(rc, pathEff);
            paint.setPathEffect(p);
        }
        else
        {
            paint.setPathEffect(NULL);
        }
    }

    int color = req->getIntPropertyValue(rColor);
    paint.setColor(color);

    if (ind == 0)
    {
        std::string shader = req->getStringPropertyValue(req->props()->R_SHADER);
        if (shader.size() > 0)
        {
            SkBitmap* bmp = getCachedBitmap(rc, shader);
            if (bmp != NULL) {
                paint.setShader(new SkBitmapProcShader(*bmp, SkShader::kRepeat_TileMode, SkShader::kRepeat_TileMode))->unref();
                if(color == 0) {
					paint.setColor(0xffffffff);
                }
            }
        }
    }

    // do not check shadow color here
    if (rc.getShadowRenderingMode() == 1 && ind == 0)
    {
        SkColor shadowColor = req->getIntPropertyValue(req->props()->R_SHADOW_COLOR);
        int shadowLayer = getDensityValue(rc, req, req->props()->R_SHADOW_RADIUS);
        if (shadowColor == 0) {
			shadowColor = rc.getShadowRenderingColor();
		}
        if (shadowColor == 0)
            shadowLayer = 0;

        if (shadowLayer > 0)
            paint.setLooper(new SkBlurDrawLooper(shadowLayer, 0, 0, shadowColor))->unref();
    }
    return 1;
}

void renderText(MapDataObject* obj, RenderingRuleSearchRequest* req, RenderingContext & rc, std::string const & tag,
		std::string const & value, float xText, float yText, SkPath const * path) {
	UNORDERED(map)<std::string, std::string>::const_iterator it, next = obj->objectNames.begin();
	while (next != obj->objectNames.end()) {
		it = next++;
		if (it->second.length() > 0) {
			std::string name = it->second;
			std::string tagName = it->first == "name" ? "" : it->first;
			if (tagName == "" && rc.isUsingEnglishNames() && obj->objectNames.find("name:en") !=
					obj->objectNames.end()) {
				continue;
			} 
			if (tagName == "name:en" && !rc.isUsingEnglishNames()) {
				continue;
			}
			name =rc.getTranslatedString(name);
			name =rc.getReshapedString(name);
			req->setInitialTagValueZoom(tag, value, rc.getZoom(), obj);
			req->setIntFilter(req->props()->R_TEXT_LENGTH, name.length());			
			req->setStringFilter(req->props()->R_NAME_TAG, tagName);
			if (req->searchRule(RenderingRulesStorage::TEXT_RULES)
					&& req->isSpecified(req->props()->R_TEXT_SIZE)) {
				TextDrawInfo* info = new TextDrawInfo(name);
				std::string tagName2 = req->getStringPropertyValue(req->props()->R_NAME_TAG2);
				if(tagName2 != "") {
					std::string tv = obj->objectNames[tagName2];
					if(tv != "") {
						info->text = name + " " + tv;
					}
				}
				info->drawOnPath = (path != NULL) && (req->getIntPropertyValue(req->props()->R_TEXT_ON_PATH, 0) > 0);
				if (path != NULL)
					info->path = new SkPath(*path);

				fillTextProperties(rc, info, req, xText, yText);
				rc.textToDraw.push_back(info);
			}
		}
	}
}

void drawPolylineShadow(SkCanvas & cv, SkPaint & paint, RenderingContext const & rc, SkPath const & path,
		SkColor shadowColor, int shadowRadius)
{
    // blurred shadows
    if (rc.getShadowRenderingMode() == 2 && shadowRadius > 0) {
        // simply draw shadow? difference from option 3 ?
        // paint->setColor(0xffffffff);
        paint.setLooper(new SkBlurDrawLooper(shadowRadius, 0, 0, shadowColor))->unref();
        PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
    }

    // option shadow = 3 with solid border
    if (rc.getShadowRenderingMode() == 3 && shadowRadius > 0) {
        paint.setLooper(NULL);
        paint.setStrokeWidth(paint.getStrokeWidth() + shadowRadius * 2);
        //		paint->setColor(0xffbababa);
        paint.setColorFilter(SkColorFilter::CreateModeFilter(shadowColor, SkXfermode::kSrcIn_Mode))->unref();
        //		paint->setColor(shadowColor);
        PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
    }
}

SkPaint* oneWayPaint(){
    SkPaint* oneWay = new SkPaint;
    oneWay->setStyle(SkPaint::kStroke_Style);
    oneWay->setColor(0xff6c70d5);
    oneWay->setAntiAlias(true);
    return oneWay;
}

void drawOneWayPaints(RenderingContext & rc, SkCanvas & cv, SkPath const & p, int oneway) {
	float rmin = rc.getDensityValue(1);
	if(rmin > 1) {
		rmin = rmin * 2 / 3;
	}
	if (rc.oneWayPaints.size() == 0) {
        const float intervals_oneway[4][4] = {
            {0, 12, 10 * rmin, 152},
            {0, 12,  9 * rmin, 152 + rmin},
            {0, 12 + 6 * rmin, 2 * rmin , 152 + 2 * rmin},
            {0, 12 + 6 * rmin, 1 * rmin, 152 + 3 * rmin}
        };
		SkPathEffect* arrowDashEffect1 = new SkDashPathEffect(intervals_oneway[0], 4, 0);
		SkPathEffect* arrowDashEffect2 = new SkDashPathEffect(intervals_oneway[1], 4, 1);
		SkPathEffect* arrowDashEffect3 = new SkDashPathEffect(intervals_oneway[2], 4, 1);
		SkPathEffect* arrowDashEffect4 = new SkDashPathEffect(intervals_oneway[3], 4, 1);

		////// TODO oneWayPaint?????
		SkPaint* p = oneWayPaint();
		p->setStrokeWidth(rmin);
		p->setPathEffect(arrowDashEffect1)->unref();
		rc.oneWayPaints.push_back(*p);
		delete p;

		p = oneWayPaint();
		p->setStrokeWidth(rmin * 2);
		p->setPathEffect(arrowDashEffect2)->unref();
		rc.oneWayPaints.push_back(*p);
		delete p;

		p = oneWayPaint();
		p->setStrokeWidth(rmin * 3);
		p->setPathEffect(arrowDashEffect3)->unref();
		rc.oneWayPaints.push_back(*p);
		delete p;

		p = oneWayPaint();
		p->setStrokeWidth(rmin * 4);
		p->setPathEffect(arrowDashEffect4)->unref();
		rc.oneWayPaints.push_back(*p);
		delete p;
	}
	
	if (rc.reverseWayPaints.size() == 0) {
            const float intervals_reverse[4][4] = {
                {0, 12, 10 * rmin, 152},
                {0, 12 + 1 * rmin, 9 * rmin, 152},
                {0, 12 + 2 * rmin, 2 * rmin, 152 + 6 * rmin},
                {0, 12 + 3 * rmin, 1 * rmin, 152 + 6 * rmin}
            };            
		SkPathEffect* arrowDashEffect1 = new SkDashPathEffect(intervals_reverse[0], 4, 0);
		SkPathEffect* arrowDashEffect2 = new SkDashPathEffect(intervals_reverse[1], 4, 1);
		SkPathEffect* arrowDashEffect3 = new SkDashPathEffect(intervals_reverse[2], 4, 1);
		SkPathEffect* arrowDashEffect4 = new SkDashPathEffect(intervals_reverse[3], 4, 1);
		SkPaint* p = oneWayPaint();
		p->setStrokeWidth(rmin * 1);
		p->setPathEffect(arrowDashEffect1)->unref();
		rc.reverseWayPaints.push_back(*p);
		delete p;

		p = oneWayPaint();
		p->setStrokeWidth(rmin * 2);
		p->setPathEffect(arrowDashEffect2)->unref();
		rc.reverseWayPaints.push_back(*p);
		delete p;

		p = oneWayPaint();
		p->setStrokeWidth(rmin * 3);
		p->setPathEffect(arrowDashEffect3)->unref();
		rc.reverseWayPaints.push_back(*p);
		delete p;

		p = oneWayPaint();
		p->setStrokeWidth(rmin * 4);
		p->setPathEffect(arrowDashEffect4)->unref();
		rc.reverseWayPaints.push_back(*p);
		delete p;
	}
	if (oneway > 0) {
		for (size_t i = 0; i < rc.oneWayPaints.size(); i++) {
			PROFILE_NATIVE_OPERATION(rc, cv.drawPath(p, rc.oneWayPaints[i]));
		}
	} else {
		for (size_t i = 0; i < rc.reverseWayPaints.size(); i++) {
			PROFILE_NATIVE_OPERATION(rc, cv.drawPath(p, rc.reverseWayPaints[i]));
		}
	}
}

void drawPolyline(MapDataObject* mObj, RenderingRuleSearchRequest* req, SkCanvas & cv, SkPaint & paint,
	RenderingContext & rc, tag_value const & pair, int layer, int drawOnlyShadow) {
	size_t length = mObj->points.size();
	if (length < 2) {
		return;
	}
	std::string const & tag = pair.first;
	std::string const & value = pair.second;

	req->setInitialTagValueZoom(tag, value, rc.getZoom(), mObj);
	req->setIntFilter(req->props()->R_LAYER, layer);
	bool rendered = req->searchRule(2);
	if (!rendered || !updatePaint(req, paint, 0, 0, rc)) {
		return;
	}
	int shadowColor = req->getIntPropertyValue(req->props()->R_SHADOW_COLOR);
	int shadowRadius = req->getIntPropertyValue(req->props()->R_SHADOW_RADIUS);
	if(drawOnlyShadow && shadowRadius == 0) {
		return;
	}
	if(shadowColor == 0) {
		shadowColor = rc.getShadowRenderingColor();
	}
	int oneway = 0;
	if (rc.getZoom() >= 16 && pair.first == "highway") {
		if (mObj->containsAdditional("oneway", "yes")) {
			oneway = 1;
		} else if (mObj->containsAdditional("oneway", "-1")) {
			oneway = -1;
		}
	}

	rc.visible++;
	SkPath path;
	SkPoint middlePoint;
	bool intersect = false;
	uint prevCross = 0;
	uint middle = length / 2;
	uint i = 0;
	for (; i < length; i++) {
		calcPoint(mObj->points.at(i), rc);
		if (i == 0) {
			path.moveTo(rc.calcX, rc.calcY);
		} else {
			if (i == middle) {
				middlePoint.set(rc.calcX, rc.calcY);
			}
			path.lineTo(rc.calcX, rc.calcY);
		}
		if (!intersect) {
			if (rc.calcX >= 0 && rc.calcY >= 0 && rc.calcX < rc.getWidth() && rc.calcY < rc.getHeight()) {
				intersect = true;
			} else {
				uint cross = 0;
				cross |= (rc.calcX < 0 ? 1 : 0);
				cross |= (rc.calcX > rc.getWidth() ? 2 : 0);
				cross |= (rc.calcY < 0 ? 4 : 0);
				cross |= (rc.calcY > rc.getHeight() ? 8 : 0);
				if(i > 0) {
					if((prevCross & cross) == 0) {
						intersect = true;
					}
				}
				prevCross = cross;
			}
		}
	}

	if (!intersect) {
		return;
	}

	if (i > 0) {
		if (drawOnlyShadow) {
			drawPolylineShadow(cv, paint, rc, path, shadowColor, shadowRadius);
		} else {
			if (updatePaint(req, paint, -2, 0, rc)) {
				PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
			}
			if (updatePaint(req, paint, -1, 0, rc)) {
				PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
			}
			if (updatePaint(req, paint, 0, 0, rc)) {
				PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
			}
			PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
			if (updatePaint(req, paint, 1, 0, rc)) {
				PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
			}
			if (updatePaint(req, paint, 2, 0, rc)) {
				PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
			}
			if (updatePaint(req, paint, 3, 0, rc)) {
				PROFILE_NATIVE_OPERATION(rc, cv->drawPath(path, *paint));
			}
			if (oneway && !drawOnlyShadow) {
				drawOneWayPaints(rc, cv, path, oneway);
			}
			if (!drawOnlyShadow) {
				renderText(mObj, req, rc, pair.first, pair.second, middlePoint.fX, middlePoint.fY, &path);
			}
		}
	}
}
#define I_MIN_VALUE 0x8000

int ray_intersect_xo(int prevX, int prevY, int x, int y, int middleY) {
	// prev node above line
	// x,y node below line
	if (prevY > y) {
		int tx = x;
		int ty = y;
		x = prevX;
		y = prevY;
		prevX = tx;
		prevY = ty;
	}
	if (y == middleY || prevY == middleY) {
			middleY -= 1;
	}
	if (prevY > middleY || y < middleY) {
			return I_MIN_VALUE;
	} else {
		if (y == prevY) {
			// the node on the boundary !!!
			return x;
		}
		return x + (middleY - y) * (x - prevX) / (y - prevY);
	}
}
// COPIED from MapAlgorithms
bool ray_intersect_x(int prevX, int prevY, int nx, int ny, int x, int y) {
	int t = ray_intersect_xo(prevX, prevY, nx, ny, y);
	if(t == I_MIN_VALUE){
		return false;
	}
	if(t < x){
		return true;
	}
	return false;
}

int countIntersections(std::vector<std::pair<int,int> > const & points, int x, int y) {
	int intersections = 0;
	for (size_t i = 0; i < points.size() - 1; i++) {
		if (ray_intersect_x(points[i].first, points[i].second,
				points[i + 1].first, points[i + 1].second, x, y)) {
			intersections++;
		}
	}
	// special handling, also count first and last, might not be closed, but
	// we want this!
	if (ray_intersect_x(points[0].first, points[0].second,
				points[points.size()-1].first, points[points.size()-1].second, x, y)) {
		intersections++;
	}
	return intersections;
}

bool contains(std::vector<std::pair<int,int> > const & points, int x, int y) {
	return countIntersections(points, x, y) % 2 == 1;
}

void drawPolygon(MapDataObject* mObj, RenderingRuleSearchRequest* req, SkCanvas & cv, SkPaint & paint,
	RenderingContext & rc, tag_value const & pair) {
	size_t length = mObj->points.size();
	if (length <= 2) {
		return;
	}
	std::string const & tag = pair.first;
	std::string const & value = pair.second;

	req->setInitialTagValueZoom(tag, value, rc.getZoom(), mObj);
	bool rendered = req->searchRule(3);

	float xText = 0;
	float yText = 0;
	if (!rendered || !updatePaint(req, paint, 0, 1, rc)) {
		return;
	}

	rc.visible++;
	SkPath path;
	bool containsPoint = false;
	int bounds = 0;
	std::vector< std::pair<int,int > > ps;
	for (size_t i = 0; i < length; i++) {
		calcPoint(mObj->points[i], rc);
		if (i == 0) {
			path.moveTo(rc.calcX, rc.calcY);
		} else {
			path.lineTo(rc.calcX, rc.calcY);
		}
		float tx = rc.calcX;
		if (tx < 0) {
			tx = 0;
		}
		if (tx > rc.getWidth()) {
			tx = rc.getWidth();
		}
		float ty = rc.calcY;
		if (ty < 0) {
			ty = 0;
		}
		if (ty > rc.getHeight()) {
			ty = rc.getHeight();
		}
		xText += tx;
		yText += ty;
		if (!containsPoint) {
			if (rc.calcX >= 0 && rc.calcY >= 0 && rc.calcX < rc.getWidth() && rc.calcY < rc.getHeight()) {
				containsPoint = true;
			} else {
				ps.push_back(std::pair<int, int>(rc.calcX, rc.calcY));
			}
			bounds |= (rc.calcX < 0 ? 1 : 0);
			bounds |= (rc.calcX >= rc.getWidth() ? 2 : 0);
			bounds |= (rc.calcY >= rc.getHeight()  ? 4 : 0);
			bounds |= (rc.calcY <= rc.getHeight() ? 8 : 0);
		}
	}
	xText /= length;
	yText /= length;
	if(!containsPoint){
		// fast check for polygons
		if(((bounds & 3) != 3) || ((bounds >> 2) != 3) ) {
			return;
		}
		if(contains(ps, 0, 0) ||
			contains(ps, rc.getWidth(), rc.getHeight()) ||
			contains(ps, 0, rc.getHeight()) ||
			contains(ps, rc.getWidth(), 0)) {
			if(contains(ps, rc.getWidth() / 2, rc.getHeight() / 2)) {
				xText = rc.getWidth() / 2;
				yText = rc.getHeight() / 2;
			}
		} else {
			return;
		}
	}
	std::vector<coordinates> const & polygonInnerCoordinates = mObj->polygonInnerCoordinates;
	if (!polygonInnerCoordinates.empty()) {
		path.setFillType(SkPath::kEvenOdd_FillType);
		for (size_t j = 0; j < polygonInnerCoordinates.size(); j++) {
			coordinates const & cs = polygonInnerCoordinates[j];
			if (!cs.empty())
			{
				calcPoint(cs[0], rc);
				path.moveTo(rc.calcX, rc.calcY);
				for (size_t i = 1; i < cs.size(); i++) {
					calcPoint(cs[i], rc);
					path.lineTo(rc.calcX, rc.calcY);
				}
			}
		}
	}

	PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
	if (updatePaint(req, paint, 1, 0, rc)) {
		PROFILE_NATIVE_OPERATION(rc, cv.drawPath(path, paint));
	}

	renderText(mObj, req, rc, pair.first, pair.second, xText, yText, NULL);
}

void drawPoint(MapDataObject* mObj, RenderingRuleSearchRequest* req, SkCanvas & cv, SkPaint const & paint,
	RenderingContext & rc, std::pair<std::string, std::string> const & pair, bool renderTxt)
{
	std::string const & tag = pair.first;
	std::string const & value = pair.second;

	req->setInitialTagValueZoom(tag, value, rc.getZoom(), mObj);
	req->searchRule(1);
	std::string const & resId = req->getStringPropertyValue(req-> props()-> R_ICON);
	SkBitmap* bmp = getCachedBitmap(rc, resId);
	
	if (bmp == NULL && !renderTxt)
		return;
	
	size_t length = mObj->points.size();
	rc.visible++;
	float px = 0;
	float py = 0;
	for (size_t i = 0; i < length; i++) {
		calcPoint(mObj->points[i], rc);
		px += rc.calcX;
		py += rc.calcY;
	}
	px /= length;
	py /= length;

	if (bmp != NULL) {
		IconDrawInfo ico;
		ico.x = px;
		ico.y = py;
		ico.bmp = bmp;
		ico.order = req->getIntPropertyValue(req-> props()-> R_ICON_ORDER, 100);
		rc.iconsToDraw.push_back(ico);
	}
	if (renderTxt) {
		renderText(mObj, req, rc, pair.first, pair.second, px, py, NULL);
	}
}

void drawObject(RenderingContext & rc, SkCanvas & cv, RenderingRuleSearchRequest* req,
	SkPaint & paint, std::vector<MapDataObjectPrimitive>& array, int objOrder) {

	for (size_t i = 0; i < array.size(); i++) {
		rc.allObjects++;
		MapDataObject* mObj = array[i].obj;
		tag_value const & pair = mObj->types[array[i].typeInd];
		if (objOrder == 0) {
			// polygon
			drawPolygon(mObj, req, cv, paint, rc, pair);
		} else if (objOrder == 1 || objOrder == 2) {
			drawPolyline(mObj, req, cv, paint, rc, pair, mObj->getSimpleLayer(), objOrder == 1);
		} else if (objOrder == 3) {
			drawPoint(mObj, req, cv, paint, rc, pair, array[i].typeInd == 0);
		}
		if (i % 25 == 0 && rc.interrupted()) {
			return;
		}
	}
}

bool iconOrder(IconDrawInfo const & text1, IconDrawInfo const & text2) {
	return text1.order < text2.order;
}

void drawIconsOverCanvas(RenderingContext & rc, SkCanvas & canvas)
{
	std::sort(rc.iconsToDraw.begin(), rc.iconsToDraw.end(), iconOrder);
	SkRect bounds = SkRect::MakeLTRB(0, 0, rc.getWidth(), rc.getHeight());
	bounds.inset(-bounds.width()/4, -bounds.height()/4);
	quad_tree<SkRect> boundsIntersect(bounds, 4, 0.6);
	size_t ji = 0;
	SkPaint p;
	p.setStyle(SkPaint::kStroke_Style);
	p.setFilterBitmap(true);
	std::vector<SkRect> searchText;
	for(;ji< rc.iconsToDraw.size(); ji++)
	{
		IconDrawInfo icon = rc.iconsToDraw[ji];
		if (icon.y >= 0 && icon.y < rc.getHeight() && icon.x >= 0 && icon.x < rc.getWidth() && icon.bmp != NULL) {
			SkBitmap* ico = icon.bmp;
			float left = icon.x -  ico->width() / 2 * rc.getScreenDensityRatio();
			float top = icon.y - ico->height() / 2 * rc.getScreenDensityRatio(); 
			SkRect r = SkRect::MakeXYWH(left, top, ico->width() * rc.getScreenDensityRatio(),
						ico->height()* rc.getScreenDensityRatio() );
			
			boundsIntersect.query_in_box(r, searchText);
			bool intersects = false;
			for (uint32_t i = 0; i < searchText.size(); i++) {
				SkRect& t = searchText[i];
				if (SkRect::Intersects(t, r)) {
					intersects =  true;
					break;
				}
			}
			if (!intersects) {
				PROFILE_NATIVE_OPERATION(rc, canvas.drawBitmapRect(*ico, (SkIRect*) NULL, r, &p));
				r.inset(-r.width()/4, -r.height()/4);
				boundsIntersect.insert(r, r);
			}
		}
		if (rc.interrupted()) {
			break;
		}
	}
}

double polygonArea(MapDataObject const * obj, float mult) {
	double area = 0.;
	int i0 = obj->points.size()-1;
	for (int i1 = 0; i1 < obj->points.size(); i1++) {
		int_pair const & x0 = obj->points[i0];
		int_pair const & x1 = obj->points[i1] ;
		area += (x0.first + x1.first) * (x0.second - x1.second);
		i0 = i1;
	}
	return std::abs(area)/2 * mult * mult;
}

void filterLinesByDensity(RenderingContext const & rc, std::vector<MapDataObjectPrimitive>& linesResArray,
		std::vector<MapDataObjectPrimitive> const & linesArray) {
	int roadsLimit = rc.roadsDensityLimitPerTile;
	int densityZ = rc.roadDensityZoomTile;
	if(densityZ == 0 || roadsLimit == 0) {
		linesResArray = linesArray;
		return;
	}
	linesResArray.reserve(linesArray.size());
	UNORDERED(map)<int64_t, int> densityMap;
	int dz = rc.getZoom() + densityZ;
	// From greater to lower order
	for (int i = linesArray.size() - 1; i >= 0; i--) {
		bool accept = true;
		MapDataObject const * line = linesArray[i].obj;
		tag_value const & ts = line->types[linesArray[i].typeInd];
		if (ts.first == "highway") {
			accept = false;
			int64_t prev = 0;
			for (size_t k = 0; k < line->points.size(); k++) {
				int64_t x = (line->points[k].first) >> (31 - dz);
				int64_t y = (line->points[k].second) >> (31 - dz);
				int64_t tl = (x << dz) + y;
				if (prev != tl) {
					prev = tl;
					accept = (densityMap[tl]++ < roadsLimit) || accept;
				}
			}
		}
		if(accept) {
			linesResArray.push_back(linesArray[i]);
		}
	}
	reverse(linesResArray.begin(), linesResArray.end());
}

inline bool sortByOrder(const MapDataObjectPrimitive& i,const MapDataObjectPrimitive& j) {
	return (i.order<j.order);
}

void sortObjectsByProperOrder(std::vector <MapDataObject* > const & mapDataObjects,
	RenderingRuleSearchRequest* req, RenderingContext & rc,
		std::vector<MapDataObjectPrimitive>& polygonsArray, std::vector<MapDataObjectPrimitive>& pointsArray,
		std::vector<MapDataObjectPrimitive>& linesResArray) {
	if (req != NULL) {
		std::vector<MapDataObjectPrimitive> linesArray;
		polygonsArray.reserve(20);  // Test to reduce reallocations
		linesArray.reserve(20);
		pointsArray.reserve(20);
		req->clearState();
		const int size = mapDataObjects.size();
		float mult = 1. / getPowZoom(std::max(31 - (rc.getZoom() + 8), 0));
		int i = 0;
		for (; i < size; i++) {
			MapDataObject* mobj = mapDataObjects[i];
			size_t sizeTypes = mobj->types.size();
			size_t j = 0;
			for (; j < sizeTypes; j++) {
				int layer = mobj->getSimpleLayer();
				tag_value const & pair = mobj->types[j];
				req->setTagValueZoomLayer(pair.first, pair.second, rc.getZoom(), layer, mobj);
				req->setIntFilter(req->props()->R_AREA, mobj->area);
				req->setIntFilter(req->props()->R_POINT, mobj->points.size() == 1);
				req->setIntFilter(req->props()->R_CYCLE, mobj->cycle());
				if (req->searchRule(RenderingRulesStorage::ORDER_RULES)) {
					int objectType = req->getIntPropertyValue(req->props()->R_OBJECT_TYPE);
					int order = req->getIntPropertyValue(req->props()->R_ORDER);
					MapDataObjectPrimitive mapObj;
					mapObj.objectType = objectType;
					mapObj.order = order;
					mapObj.typeInd = j;
					mapObj.obj = mobj;
					// polygon
					if(objectType == 3) {
						double area = polygonArea(mobj, mult);
						// filtering here ASAP by min size.
						if ((area > rc.polygonMinSizeToDisplay)) {
							MapDataObjectPrimitive pointObj = mapObj;
							pointObj.objectType = 1;
							polygonsArray.push_back(std::move(mapObj));
							pointsArray.push_back(std::move(pointObj)); // TODO fix duplicate text? verify if it is needed for icon
						}
					} else if(objectType == 1) {
						pointsArray.push_back(std::move(mapObj));
					} else {
						linesArray.push_back(std::move(mapObj));
					}
					if (req->getIntPropertyValue(req->props()->R_SHADOW_LEVEL) > 0) {
						rc.shadowLevelMin = std::min(rc.shadowLevelMin, order);
						rc.shadowLevelMax = std::max(rc.shadowLevelMax, order);
						req->clearIntvalue(req->props()->R_SHADOW_LEVEL);
					}
				}
			}
		}
		sort(polygonsArray.begin(), polygonsArray.end(), sortByOrder);
		sort(pointsArray.begin(), pointsArray.end(), sortByOrder);
		sort(linesArray.begin(), linesArray.end(), sortByOrder);
		filterLinesByDensity(rc, linesResArray, linesArray);
	}
}

void doRendering(std::vector <MapDataObject* > const & mapDataObjects, SkCanvas & canvas,
		RenderingRuleSearchRequest* req,
		RenderingContext & rc) {
	rc.nativeOperations.Start();
	SkPaint paint;
	paint.setAntiAlias(true);

	std::vector<MapDataObjectPrimitive>  polygonsArray;
	std::vector<MapDataObjectPrimitive>  pointsArray;
	std::vector<MapDataObjectPrimitive>  linesArray;
	sortObjectsByProperOrder(mapDataObjects, req, rc, polygonsArray, pointsArray, linesArray);
	rc.lastRenderedKey = 0;

	drawObject(rc, canvas, req, paint, polygonsArray, 0);
	rc.lastRenderedKey = 5;
	if (rc.getShadowRenderingMode() > 1) {
		drawObject(rc, canvas, req, paint, linesArray, 1);
	}
	rc.lastRenderedKey = 40;
	drawObject(rc, canvas, req, paint, linesArray, 2);
	rc.lastRenderedKey = 60;

	drawObject(rc, canvas, req, paint, pointsArray, 3);
	rc.lastRenderedKey = 125;

	drawIconsOverCanvas(rc, canvas);

	rc.textRendering.Start();
	drawTextOverCanvas(rc, canvas);
	rc.textRendering.Pause();

	rc.nativeOperations.Pause();
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,  "Native ok (rendering %d, text %d ms) \n (%d points, %d points inside, %d of %d objects visible)\n",
				(int)rc.nativeOperations.GetElapsedMs(), (int)rc.textRendering.GetElapsedMs(),
				rc.pointCount, rc.pointInsideCount, rc.visible, rc.allObjects);
}
