using namespace std;

class ColoredPoint
{
	public:
	
	double x,y,z;
	int r,g,b;
	
	ColoredPoint() {
		x = y = z = 0.0; 
		r = g = b = 0;
	}

	ColoredPoint(double x, double y, double z,int r,int g,int b) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->r = r;
		this->g = g;
		this->b = b; 
	}

	void clear() {
		x = y = z = 0.0;
		r = b = g = 0;
	}

	~ColoredPoint() {}

	ColoredPoint operator+( const ColoredPoint& other ) const {
	        return ColoredPoint(x + other.x, y + other.y, z + other.z, r + other.r, g + other.g, b + other.b);
    	}

	ColoredPoint operator-( const ColoredPoint& other ) const {
	        return ColoredPoint(x - other.x, y - other.y, z - other.z, r - other.r, g - other.g, b - other.b);
    	}

	ColoredPoint componentwiseProduct(const ColoredPoint& other) const {
	        return ColoredPoint(x * other.x, y * other.y, z * other.z, r * other.r, g * other.g, b * other.b);
	}	

	ColoredPoint operator*( const double& other ) const {
        	return ColoredPoint(x * other, y * other, z * other, r * other, g * other, b * other);
	}

	ColoredPoint operator/( const double& other ) const {
        	return ColoredPoint(x / other, y / other, z / other, r / other, g / other, b / other);
	}

	ColoredPoint minim(const ColoredPoint& other) const {
		if ((x == 0) && (y == 0) && (z == 0)) return other;
		if ((other.x == 0) && (other.y == 0) && (other.z == 0)) return ColoredPoint(x,y,z,r,g,b);
		return ColoredPoint(min(x, other.x), min(y, other.y), min(z, other.z), min(r, other.r), min(g, other.g), min(b, other.b));
	}

	ColoredPoint maxim(const ColoredPoint& other) const {
        	return ColoredPoint(max(x, other.x), max(y, other.y), max(z, other.z), max(r, other.r), max(g, other.g), max(b, other.b));
	}



};
