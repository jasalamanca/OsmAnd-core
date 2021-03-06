#ifndef _OSMAND_COMMON_H
#define _OSMAND_COMMON_H

#include <string>
#include <vector>

#include <SkPath.h>
#include <SkPaint.h>
#include <SkBitmap.h>

#include <ElapsedTimer.h>

// M_PI is no longer part of math.h/cmath by standart, but some GCC's define them
#define _USE_MATH_DEFINES
#include <math.h>
#if !defined(M_PI)
	const double M_PI = 3.14159265358979323846;
#endif
#if !defined(M_PI_2)
	const double M_PI_2 = M_PI / 2.0;
#endif

inline double toRadians(double angdeg) {
	return angdeg / 180 * M_PI;
}

struct TextDrawInfo {
	TextDrawInfo(std::string);
	~TextDrawInfo();

	std::string text;

	SkRect bounds;
	float centerX;
	float centerY;

	float textSize;
	float minDistance;
	int textColor;
	int textShadow;
	int textShadowColor;
	uint32_t textWrap;
	bool bold;
	std::string shieldRes;
	int textOrder;

	bool drawOnPath;
	SkPath* path;
	float pathRotate;
	float vOffset;
	float hOffset;
};

struct IconDrawInfo
{
	IconDrawInfo();

	SkBitmap* bmp;
	float x;
	float y;
	int order;
};

static const int TILE_SIZE = 256;
struct RenderingContext
{
private :
	// parameters
	bool useEnglishNames;
	float density;
	float screenDensityRatio;

	double leftX;
	double topY;
	int width;
	int height;
	int defaultColor;

	int zoom;
	float rotate;
	// int shadowRenderingMode = 0; // no shadow (minumum CPU)
	// int shadowRenderingMode = 1; // classic shadow (the implementaton in master)
	// int shadowRenderingMode = 2; // blur shadow (most CPU, but still reasonable)
	// int shadowRenderingMode = 3; solid border (CPU use like classic version or even smaller)
	int shadowRenderingMode;
	int shadowRenderingColor;
	std::string defaultIconsDir;

public:
	// debug purpose
	int pointCount;
	int pointInsideCount;
	int visible;
	int allObjects;
	int lastRenderedKey;
	OsmAnd::ElapsedTimer textRendering;
	OsmAnd::ElapsedTimer nativeOperations;

	std::vector<SkPaint> oneWayPaints;
	std::vector<SkPaint> reverseWayPaints;

// because they used in 3rd party functions
public :

	// calculated
	double tileDivisor;
	float cosRotateTileSize;
	float sinRotateTileSize;

	// use to calculate points
	float calcX;
	float calcY;

	std::vector<TextDrawInfo*> textToDraw;
	std::vector<IconDrawInfo> iconsToDraw;

	// not expect any shadow
	int shadowLevelMin;
	int shadowLevelMax;
	int polygonMinSizeToDisplay;
	int roadDensityZoomTile;
	int roadsDensityLimitPerTile;

public:
	RenderingContext() : useEnglishNames(false), density(1), screenDensityRatio(1),
			//leftX, topY, width, height
			defaultColor(0xfff1eee8), zoom(15), rotate(0),
			shadowRenderingMode(2), shadowRenderingColor(0xff969696), // defaultIconsDir
			pointCount(0), pointInsideCount(0), visible(0), allObjects(0), lastRenderedKey(0),
			// textRendering, nativeOperations, oneWayPaints, reverseWayPaints
			// tileDivisor, cosRotateTileSize, sinRotateTileSize,  calcX, calcY
			// textToDraw, iconsToDraw,
			shadowLevelMin(256), shadowLevelMax(0), polygonMinSizeToDisplay(75),
			roadDensityZoomTile(0), roadsDensityLimitPerTile(0) 
	{
	}
	virtual ~RenderingContext();

	virtual bool interrupted() const;
	virtual SkBitmap* getCachedBitmap(const std::string& bitmapResource) const;
	virtual std::string const & getTranslatedString(const std::string& src) const;
	virtual std::string const & getReshapedString(const std::string& src) const;

	void setDefaultIconsDir(std::string const & path) {
		defaultIconsDir = path;
	}

	void setZoom(int z) {
		this->zoom = z;
		this->tileDivisor = (1 << (31 - z));
	}

	void setTileDivisor(double tileDivisor) {
		this->tileDivisor = tileDivisor;
	}

	void setDefaultColor(int z) {
		this->defaultColor = z;
	}

	void setRotate(float rot) {
		this->rotate = rot;
		this->cosRotateTileSize = cos(toRadians(rot)) * TILE_SIZE;
		this->sinRotateTileSize = sin(toRadians(rot)) * TILE_SIZE;
	}

	void setLocation(double leftX, double topY) {
		this->leftX = leftX;
		this->topY = topY;
	}

	void setDimension(int width, int height) {
		this->width = width;
		this->height = height;
	}

	inline int getShadowRenderingMode() const {
		return shadowRenderingMode;
	}

	int getShadowRenderingColor() const {
		return shadowRenderingColor;
	}

	void setShadowRenderingColor(int color) {
		shadowRenderingColor = color;
	}

	inline int getWidth() const {
		return width;
	}

	inline int getDefaultColor() const {
		return defaultColor;
	}

	inline int getHeight() const {
		return height;
	}

	inline int getZoom() const {
		return zoom;
	}

	inline double getLeft() {
		return leftX;
	}

	inline double getTop() {
		return topY;
	}

	void setShadowRenderingMode(int mode){
		this->shadowRenderingMode = mode;
	}

	void setDensityScale(float val) {
		density = val;
	}

	void setScreenDensityRatio(float v)  {
		screenDensityRatio = v;
	}

	float getScreenDensityRatio() const {
		return screenDensityRatio;
	}

	float getDensityValue(float val) const {
		return val * density;
	}

	float getDensityValue(float val, int pxValues) const {
		return val * density + pxValues;
	}

	void setUseEnglishNames(bool b){
		this->useEnglishNames = b;
	}

	bool isUsingEnglishNames() const {
		return this->useEnglishNames;
	}
};

SkBitmap* getCachedBitmap(RenderingContext & rc, const std::string& bitmapResource);
void purgeCachedBitmaps();

int get31TileNumberX(double longitude);
int get31TileNumberY( double latitude);

double getPowZoom(float zoom);

double getLongitudeFromTile(float zoom, double x) ;
double getLatitudeFromTile(float zoom, double y);

double get31LongitudeX(int tileX);
double get31LatitudeY(int tileY);
double getTileNumberX(float zoom, double longitude);
double getTileNumberY(float zoom, double latitude);
double getDistance(double lat1, double lon1, double lat2, double lon2);
double getPowZoom(float zoom);

double calculateProjection31TileMetric(int xA, int yA, int xB, int yB, int xC, int yC);
std::pair<int, int> calculateProjectionPoint31(int xA, int yA, int xB, int yB, int xC, int yC);
double squareDist31TileMetric(int x1, int y1, int x2, int y2);
inline double distance31TileMetric(int x1, int y1, int x2, int y2)
{
	return sqrt(squareDist31TileMetric(x1, y1, x2, y2));
}
double convert31YToMeters(int y1, int y2);
double convert31XToMeters(int y1, int y2);
double alignAngleDifference(double diff);

template <typename T> class quad_tree {
private :
	struct node {
        typedef std::vector<T> cont_t;
        cont_t data;
		node* children[4];
		SkRect bounds;

		node(SkRect& b) : bounds(b) {
            memset(children,0,4*sizeof(node*));
		}

		~node() {
			for (int i = 0; i < 4; i++) {
				if (children[i] != NULL) {
					delete children[i];
				}
			}
		}
	};
	typedef typename node::cont_t cont_t;
	typedef typename cont_t::iterator node_data_iterator;
	double ratio;
	unsigned int max_depth;
	node root;
public:
	quad_tree(SkRect r=SkRect::MakeLTRB(0,0,0x7FFFFFFF,0x7FFFFFFF), int depth=8, double ratio = 0.55) : ratio(ratio), max_depth(depth), root(r) {
	}

    void insert(T data, SkRect& box)
    {
        unsigned int depth=0;
        do_insert_data(data, box, &root, depth);
    }

    void query_in_box(SkRect& box, std::vector<T>& result)
    {
        result.clear();
        query_node(box, result, &root);
    }

private:

    void query_node(SkRect& box, std::vector<T> & result, node* node) const {
		if (node) {
			if (SkRect::Intersects(box, node->bounds)) {
				node_data_iterator i = node->data.begin();
				node_data_iterator end = node->data.end();
				while (i != end) {
					result.push_back(*i);
					++i;
				}
				for (int k = 0; k < 4; ++k) {
					query_node(box, result, node->children[k]);
				}
			}
		}
	}


    void do_insert_data(T data, SkRect& box, node * n, unsigned int& depth)
    {
        if (++depth >= max_depth) {
			n->data.push_back(data);
		} else {
			SkRect& node_extent = n->bounds;
			SkRect ext[4];
			split_box(node_extent, ext);
			for (int i = 0; i < 4; ++i) {
				if (ext[i].contains(box)) {
					if (!n->children[i]) {
						n->children[i] = new node(ext[i]);
					}
					do_insert_data(data, box, n->children[i], depth);
					return;
				}
			}
			n->data.push_back(data);
		}
    }
    void split_box(SkRect& node_extent,SkRect * ext)
    {
        //coord2d c=node_extent.center();

    	float width=node_extent.width();
    	float height=node_extent.height();

        float lox=node_extent.fLeft;
        float loy=node_extent.fTop;
        float hix=node_extent.fRight;
        float hiy=node_extent.fBottom;

        ext[0]=SkRect::MakeLTRB(lox,loy,lox + width * ratio,loy + height * ratio);
        ext[1]=SkRect::MakeLTRB(hix - width * ratio,loy,hix,loy + height * ratio);
        ext[2]=SkRect::MakeLTRB(lox,hiy - height*ratio,lox + width * ratio,hiy);
        ext[3]=SkRect::MakeLTRB(hix - width * ratio,hiy - height*ratio,hix,hiy);
    }
};

int findFirstNumberEndIndex(std::string const & value);

#endif /*_OSMAND_COMMON_H*/
