#include <vector>
#include <set>
#include <algorithm>
#include <math.h>
#include <time.h>
#include "SkTypes.h"
#include "SkTypeface.h"
#include "SkCanvas.h"
#include "SkPaint.h"
#include "SkPath.h"

#include "Common.h"
#include "common2.h"
#include "Logging.h"
#include "renderRules.h"
//#include "utf8.cpp"
#include "utf8/unchecked.h"


inline float sqr(float a){
	return a*a;
}

inline float absFloat(float a){
	return a > 0 ? a : -a;
}

void fillTextProperties(RenderingContext const & rc, TextDrawInfo* info, RenderingRuleSearchRequest* render, float cx, float cy) {
	info->centerX = cx;
	// used only for draw on path where centerY doesn't play role
	info->vOffset = getDensityValue(rc, render, render->props()->R_TEXT_DY);
	info->centerY = cy + info->vOffset;
	info->textColor = render->getIntPropertyValue(render->props()->R_TEXT_COLOR);
	if (info->textColor == 0) {
		info->textColor = 0xff000000;
	}
	info->textSize = getDensityValue(rc, render, render->props()->R_TEXT_SIZE);
	info->textShadow = getDensityValue(rc, render, render->props()->R_TEXT_HALO_RADIUS);
	info->textShadowColor = render->getIntPropertyValue(render->props()->R_TEXT_HALO_COLOR);
	if (info->textShadowColor == 0) {
		info->textShadowColor = 0xffffffff;
	}
	info->textWrap = getDensityValue(rc, render, render->props()->R_TEXT_WRAP_WIDTH);
	info->bold = render->getIntPropertyValue(render->props()->R_TEXT_BOLD, 0) > 0;
	info->minDistance = getDensityValue(rc, render, render->props()->R_TEXT_MIN_DISTANCE);
	info->shieldRes = render->getStringPropertyValue(render->props()->R_TEXT_SHIELD);
	info->textOrder = render->getIntPropertyValue(render->props()->R_TEXT_ORDER, 100);
}

bool isLetterOrDigit(char c)
{
	return c != ' ';
}

void drawTextOnCanvas(SkCanvas & cv, const char* text, uint16_t len, float centerX, float centerY, SkPaint& paintText,
		int textShadowColor, float textShadow) {
	if (textShadow > 0) {
		int c = paintText.getColor();
		paintText.setStyle(SkPaint::kStroke_Style);
		paintText.setColor(textShadowColor); // white
		paintText.setStrokeWidth(2 + textShadow);
		cv.drawText(text, len, centerX, centerY, paintText);
// reset
		paintText.setStrokeWidth(2);
		paintText.setStyle(SkPaint::kFill_Style);
		paintText.setColor(c);
	}
	cv.drawText(text, len, centerX, centerY, paintText);
}

int nextWord(uint8_t* s, size_t* charRead) {
	uint8_t* init = s;
	while((*s) != 0) {
		uint32_t tp = utf8::unchecked::next(s);
		(*charRead) ++;
		if(tp == ' ' || tp == '\t') {
			return (s - init);
		}
	}
	return -1;
}

void drawWrappedText(RenderingContext const & rc, SkCanvas & cv, TextDrawInfo* text, float textSize, SkPaint & paintText) {
	if(text->textWrap == 0) {
		// set maximum for all text
		text->textWrap = 40;
	}

	if(text->text.length() > text->textWrap) {
		const char* c_str = text->text.c_str();

		int end = text->text.length();
		int line = 0;
		int pos = 0;
		int start = 0;
		while(start < end) {
			const char* p_str = c_str;
			size_t charRead = 0;
			do {
				int lastSpace = nextWord((uint8_t*)p_str, &charRead);
				if (lastSpace == -1) {
					pos = end;
				} else {
					p_str += lastSpace;
					if(pos != start && charRead >= text->textWrap){
						break;
					}
					pos += lastSpace;
				}
			} while(pos < end && charRead < text->textWrap);

			PROFILE_NATIVE_OPERATION(rc, drawTextOnCanvas(cv, c_str, pos - start , text->centerX, text->centerY + line * (textSize + 2), paintText, 
				text->textShadowColor, text->textShadow));
			c_str += (pos - start);
			start = pos;
			line++;
		}
	} else {
		PROFILE_NATIVE_OPERATION(rc, drawTextOnCanvas(cv, text->text.data(), text->text.length(), text->centerX, text->centerY, paintText, 
			text->textShadowColor, text->textShadow));
	}
}

bool calculatePathToRotate(RenderingContext const & rc, TextDrawInfo* p) {
	if(p->path == NULL) {
		return true;
	}
	int len = p->path->countPoints();
    SkPoint* points= new SkPoint[len];// NEVER DEALLOCATED!
	p->path->getPoints(points, len);

	bool inverse = false;
	float roadLength = 0;
	bool prevInside = false;
	float visibleRoadLength = 0;
	float textw = p->bounds.width();
	int i;
	int startVisible = 0;
	std::vector<float> distances;
	distances.resize(roadLength, 0);

	float normalTextLen = 1.5 * textw;
	for (i = 0; i < len; i++) {
		bool inside = points[i].fX >= 0 && points[i].fX <= rc.getWidth() &&
				points[i].fY >= 0 && points[i].fY <= rc.getHeight();
		if (i > 0) {
			float d = sqrt(
					(points[i].fX - points[i - 1].fX) * (points[i].fX - points[i - 1].fX)
							+ (points[i].fY - points[i - 1].fY) * (points[i].fY - points[i - 1].fY));
			distances.push_back(d);
			roadLength += d;
			if(inside) {
				visibleRoadLength += d;
				if(!prevInside) {
					startVisible = i - 1;
				}
			} else if(prevInside) {
				if(visibleRoadLength >= normalTextLen) {
					break;
				}
				visibleRoadLength = 0;
			}

		}
		prevInside = inside;
	}
	if (textw >= roadLength) {
		return false;
	}
	int startInd = 0;
	int endInd = len;

	if(textw < visibleRoadLength && i - startVisible > 1) {
		startInd = startVisible;
		endInd = i;
		// display long road name in center
		if (visibleRoadLength > 3 * textw) {
			bool ch ;
			do {
				ch = false;
				if(endInd - startInd > 2 && visibleRoadLength - distances[startInd] > normalTextLen){
					visibleRoadLength -= distances.at(startInd);
					startInd++;
					ch = true;
				}
				if(endInd - startInd > 2 && visibleRoadLength - distances[endInd - 2] > normalTextLen){
					visibleRoadLength -= distances.at(endInd - 2);
					endInd--;
					ch = true;
				}
			} while(ch);
		}
	}

	if (!p->drawOnPath) {
		float px = 0;
		float py = 0;
		for (i = startInd; i < endInd; i++) {
			px += points[i].fX ;
			py += points[i].fY;
		}
		px /= (endInd - startInd);
		py /= (endInd - startInd);
		float cx = 0;
		float cy = 0;
		float cd = -1;
		for (i = startInd + 1; i < endInd; i++) {
			float fd = sqr(px - points[i].fX) + sqr(py - points[i].fY);
			if (cd < 0 || fd < cd) {
				cx = points[i].fX;
				cy = points[i].fY;
				cd = fd;
			}
		}
		p->centerX = cx;
		p->centerY = cy;
		p->pathRotate = atan2(py, px);
		p->vOffset += p->textSize / 2 - 1;
		p->hOffset = 0;
	} else {
		// shrink path to display more text

		if (startInd > 0 || endInd < len) {
			// find subpath
			SkPath* path = new SkPath;
			for (int i = startInd; i < endInd; i++) {
				if (i == startInd) {
					path->moveTo(points[i].fX, points[i].fY);
				} else {
					path->lineTo(points[i].fX, points[i].fY);
				}
			}
			if (p->path != NULL) {
				delete p->path;
			}
			p->path = path;
		}
		// calculate vector of the road (px, py) to proper rotate it
		float px = 0;
		float py = 0;
		for (i = startInd + 1; i < endInd; i++) {
			px += points[i].fX - points[i - 1].fX;
			py += points[i].fY - points[i - 1].fY;
		}
		float scale = 0.5f;
		float plen = sqrt(px * px + py * py);
		// vector ox,oy orthogonal to px,py to measure height
		float ox = -py;
		float oy = px;
		if (plen > 0 ) {
			float rot = atan2(py, px);
			if (rot < 0)
				rot += M_PI * 2;
			if (rot > M_PI_2 && rot < 3 * M_PI_2) {
				rot += M_PI;
				inverse = true;
				ox = -ox;
				oy = -oy;
			}
			p->pathRotate = rot;
			ox *= (p->bounds.height() / plen) / 2;
			oy *= (p->bounds.height() / plen) / 2;
		}

		p->centerX = points[startInd].fX + scale * px + ox;
		p->centerY = points[startInd].fY + scale * py + oy;
		p->vOffset += p->textSize / 2 - 1;
		p->hOffset = 0;

		if (inverse) {
			SkPath* path = new SkPath;
			for (int i = endInd - 1; i >= startInd; i--) {
				if (i == (int) (endInd - 1)) {
					path->moveTo(points[i].fX, points[i].fY);
				} else {
					path->lineTo(points[i].fX, points[i].fY);
				}
			}
			if (p->path != NULL) {
				delete p->path;
			}
			p->path = path;
		}
	}
	return true;
}

void drawTestBox(SkCanvas* cv, SkRect* r, float rot, SkPaint* paintIcon, std::string text, SkPaint* paintText)
{
	cv->save();
	cv->translate(r->centerX(),r->centerY());
	cv->rotate(rot * 180 / M_PI);
	SkRect rs = SkRect::MakeLTRB(-r->width()/2, -r->height()/2,
			r->width()/2, r->height()/2);
	cv->drawRect(rs, *paintIcon);
	if (paintText != NULL) {
		cv->drawText(text.data(), text.length(), rs.centerX(), rs.centerY(),
				*paintText);
	}
	cv->restore();
}


bool intersects(SkRect tRect, float tRot, TextDrawInfo* s)
{
	float sRot = s->pathRotate;
	if (absFloat(tRot) < M_PI / 15 && absFloat(sRot) < M_PI / 15) {
		return SkRect::Intersects(tRect, s->bounds);
	}
	float dist = sqrt(sqr(tRect.centerX() - s->bounds.centerX()) + sqr(tRect.centerY() - s->bounds.centerY()));
	if(dist < 3) {
		return true;
	}
	SkRect sRect = s->bounds;

	// difference close to 90/270 degrees
	if(absFloat(cos(tRot-sRot)) < 0.3 ){
		// rotate one rectangle to 90 degrees
		tRot += M_PI_2;
		tRect = SkRect::MakeXYWH(tRect.centerX() -  tRect.height() / 2, tRect.centerY() -  tRect.width() / 2,
				tRect.height(), tRect.width());
	}

	// determine difference close to 180/0 degrees
	if(absFloat(sin(tRot-sRot)) < 0.3){
		// rotate t box
		// (calculate offset for t center suppose we rotate around s center)
		float diff = atan2(tRect.centerY() - sRect.centerY(), tRect.centerX() - sRect.centerX());
		diff -= sRot;
		float left = sRect.centerX() + dist* cos(diff) - tRect.width()/2;
		float top = sRect.centerY() - dist* sin(diff) - tRect.height()/2;
		SkRect nRect = SkRect::MakeXYWH(left, top, tRect.width(), tRect.height());
		return SkRect::Intersects(nRect, sRect);
	}

	// TODO other cases not covered
	return SkRect::Intersects(tRect, sRect);
}

bool intersects(TextDrawInfo* t, TextDrawInfo* s) {
	return intersects(t->bounds, t->pathRotate, s);
}
#if defined(WIN32)
#undef max
#endif
inline float max(float a, float b) {
  return a > b ? a : b;
}
///vector<TextDrawInfo*> searchText;
bool findTextIntersection(SkCanvas & cv, RenderingContext & rc, quad_tree<TextDrawInfo*>& boundIntersections, TextDrawInfo* text,
		SkPaint* paintText, SkPaint* paintIcon) {
	vector<TextDrawInfo*> searchText;
	paintText->measureText(text->text.c_str(), text->text.length(), &text->bounds);
	// make wider
	text->bounds.inset(-rc.getDensityValue(3), -rc.getDensityValue(10));

	bool display = calculatePathToRotate(rc, text);
	if (!display) {
		return true;
	}

	if(text->path == NULL) {
		text->bounds.offset(text->centerX, text->centerY);
		// shift to match alignment
		text->bounds.offset(-text->bounds.width()/2, 0);
	} else {
		text->bounds.offset(text->centerX - text->bounds.width()/2, text->centerY - text->bounds.height()/2);
	}

	// for text purposes
//	drawTestBox(cv, &text->bounds, text->pathRotate, paintIcon, text->text, NULL/*paintText*/);
	boundIntersections.query_in_box(text->bounds, searchText);
	for (uint32_t i = 0; i < searchText.size(); i++) {
		TextDrawInfo* t = searchText.at(i);
		if (intersects(text, t)) {
			return true;
		}
	}
	if(text->minDistance > 0) {
		SkRect boundsSearch = text->bounds;
		boundsSearch.inset(-max(rc.getDensityValue(5.0f), text->minDistance), -rc.getDensityValue(15));
		boundIntersections.query_in_box(boundsSearch, searchText);
//		drawTestBox(cv, &boundsSearch, text->pathRotate, paintIcon, text->text, paintText);
		for (uint32_t i = 0; i < searchText.size(); i++) {
			TextDrawInfo* t = searchText.at(i);
			if (t->minDistance > 0 && t->text == text->text && intersects(boundsSearch, text->pathRotate,  t)) {
				return true;
			}
		}
	}

	boundIntersections.insert(text, text->bounds);

	return false;
}


bool textOrder(TextDrawInfo* text1, TextDrawInfo* text2) {
	return text1->textOrder < text2->textOrder;
}

#if defined(ANDROID)
static SkTypeface* sDefaultTypeface = nullptr;
#endif
void drawTextOverCanvas(RenderingContext & rc, SkCanvas & cv) {
	SkRect r = SkRect::MakeLTRB(0, 0, rc.getWidth(), rc.getHeight());
	r.inset(-100, -100);
	quad_tree<TextDrawInfo*> boundsIntersect(r, 4, 0.6);

#if defined(ANDROID)
    //TODO: This is never released because of always +1 of reference counter
    if(!sDefaultTypeface)
        sDefaultTypeface = SkTypeface::CreateFromName("Droid Serif", SkTypeface::kNormal);
#endif

	SkPaint paintIcon;
	paintIcon.setStyle(SkPaint::kStroke_Style);
	paintIcon.setStrokeWidth(1);
	paintIcon.setColor(0xff000000);
	paintIcon.setFilterBitmap(true);
	SkPaint paintText;
	paintText.setStyle(SkPaint::kFill_Style);
	paintText.setStrokeWidth(1);
	paintText.setColor(0xff000000);
	paintText.setTextAlign(SkPaint::kCenter_Align);
    paintText.setAntiAlias(true);
	SkPaint::FontMetrics fm;

	// 1. Sort text using text order
	std::sort(rc.textToDraw.begin(), rc.textToDraw.end(), textOrder);
    for(auto itdi = rc.textToDraw.begin(); itdi != rc.textToDraw.end(); ++itdi)
    {
        auto textDrawInfo = *itdi;

        // Skip empty text
	    if(textDrawInfo->text.length() <= 0)
            continue;

        // Prepare font
#if defined(ANDROID)
        if(sDefaultTypeface)
            paintText.setTypeface(sDefaultTypeface);
#endif
        SkTypeface* properTypeface = nullptr;
#if defined(ANDROID)
        properTypeface = sDefaultTypeface;
#endif
        if(properTypeface)
            paintText.setTypeface(properTypeface);

		// sest text size before finding intersection (it is used there)
		float textSize = textDrawInfo->textSize;
		paintText.setTextSize(textSize);
		paintText.setFakeBoldText(textDrawInfo->bold);
		paintText.setColor(textDrawInfo->textColor);
		// align center y
		paintText.getFontMetrics(&fm);
		textDrawInfo->centerY += (-fm.fAscent);

		// calculate if there is intersection
		bool intersects = findTextIntersection(cv, rc, boundsIntersect, textDrawInfo, &paintText, &paintIcon);
		if (!intersects) {
			if(rc.interrupted()){
					return;
			}
			if (textDrawInfo->drawOnPath && textDrawInfo->path != NULL) {
				if (textDrawInfo->textShadow > 0) {
					paintText.setColor(textDrawInfo->textShadowColor);
					paintText.setStyle(SkPaint::kStroke_Style);
					paintText.setStrokeWidth(2 + textDrawInfo->textShadow);
					rc.nativeOperations.Pause();
					cv.drawTextOnPathHV(textDrawInfo->text.c_str(), textDrawInfo->text.length(), *textDrawInfo->path, textDrawInfo->hOffset,
							textDrawInfo->vOffset, paintText);
					rc.nativeOperations.Start();
					// reset
					paintText.setStyle(SkPaint::kFill_Style);
					paintText.setStrokeWidth(2);
					paintText.setColor(textDrawInfo->textColor);
				}
				rc.nativeOperations.Pause();
				cv.drawTextOnPathHV(textDrawInfo->text.c_str(), textDrawInfo->text.length(), *textDrawInfo->path, textDrawInfo->hOffset,
						textDrawInfo->vOffset, paintText);
				rc.nativeOperations.Start();
			} else {
				if (textDrawInfo->shieldRes.length() > 0) {
					SkBitmap* ico = getCachedBitmap(rc, textDrawInfo->shieldRes);
					if (ico != NULL) {
						float left = textDrawInfo->centerX - ico->width() / 2 * rc.getScreenDensityRatio()
								- 0.5f;
						float top = textDrawInfo->centerY - ico->height() / 2  * rc.getScreenDensityRatio()
								- rc.getDensityValue(4.5f);
						// SkIRect src =  SkIRect::MakeXYWH(0, 0, ico->width(), ico->height())
						SkRect r = SkRect::MakeXYWH(left, top, ico->width() * rc.getScreenDensityRatio(),
						 		ico->height() * rc.getScreenDensityRatio());
						PROFILE_NATIVE_OPERATION(rc, cv.drawBitmapRect(*ico, (SkIRect*) NULL, r, &paintIcon));
					}
				}
				drawWrappedText(rc, cv, textDrawInfo, textSize, paintText);
			}
		}
	}
}


