#define AA 1   // make this 1 is your machine is too slow
#extension GL_OES_standard_derivatives : enable

precision mediump float;


uniform vec2 u_resolution;           // viewport resolution (in pixels)
uniform float u_time;                 // shader playback time (in seconds)
uniform vec2 u_mouse;                 // shader playback time (in seconds)

const int MAX_MARCHING_STEPS = 255;
const float MIN_DIST = 0.0;
const float MAX_DIST = 500.0;
const float EPSILON = 0.0001;
const float HALF_PI = 1.5707;

float sdPlane( vec3 p )
{
	return p.y;
}

float sdSphere( vec3 p, float s )
{
    return length(p)-s;
}

float sdBox( vec3 p, vec3 b )
{
    vec3 d = abs(p) - b;
    return min(max(d.x,max(d.y,d.z)),0.0) + length(max(d,0.0));
}

float sdEllipsoid( in vec3 p, in vec3 r )
{
    return (length( p/r ) - 1.0) * min(min(r.x,r.y),r.z);
}

float udRoundBox( vec3 p, vec3 b, float r )
{
    return length(max(abs(p)-b,0.0))-r;
}

float sdTorus( vec3 p, vec2 t )
{
    return length( vec2(length(p.xz)-t.x,p.y) )-t.y;
}

float sdHexPrism( vec3 p, vec2 h )
{
    vec3 q = abs(p);
#if 0
    return max(q.z-h.y,max((q.x*0.866025+q.y*0.5),q.y)-h.x);
#else
    float d1 = q.z-h.y;
    float d2 = max((q.x*0.866025+q.y*0.5),q.y)-h.x;
    return length(max(vec2(d1,d2),0.0)) + min(max(d1,d2), 0.);
#endif
}

float sdCapsule( vec3 p, vec3 a, vec3 b, float r )
{
	vec3 pa = p-a, ba = b-a;
	float h = clamp( dot(pa,ba)/dot(ba,ba), 0.0, 1.0 );
	return length( pa - ba*h ) - r;
}

float sdEquilateralTriangle(  in vec2 p )
{
    const float k = 1.73205;//sqrt(3.0);
    p.x = abs(p.x) - 1.0;
    p.y = p.y + 1.0/k;
    if( p.x + k*p.y > 0.0 ) p = vec2( p.x - k*p.y, -k*p.x - p.y )/2.0;
    p.x += 2.0 - 2.0*clamp( (p.x+2.0)/2.0, 0.0, 1.0 );
    return -length(p)*sign(p.y);
}

float sdTriPrism( vec3 p, vec2 h )
{
    vec3 q = abs(p);
    float d1 = q.z-h.y;
#if 1
    // distance bound
    float d2 = max(q.x*0.866025+p.y*0.5,-p.y)-h.x*0.5;
#else
    // correct distance
    h.x *= 0.866025;
    float d2 = sdEquilateralTriangle(p.xy/h.x)*h.x;
#endif
    return length(max(vec2(d1,d2),0.0)) + min(max(d1,d2), 0.);
}

float sdCylinder( vec3 p, vec2 h )
{
  vec2 d = abs(vec2(length(p.xz),p.y)) - h;
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

float sdCone( in vec3 p, in vec3 c )
{
    vec2 q = vec2( length(p.xz), p.y );
    float d1 = -q.y-c.z;
    float d2 = max( dot(q,c.xy), q.y);
    return length(max(vec2(d1,d2),0.0)) + min(max(d1,d2), 0.);
}

float sdConeSection( in vec3 p, in float h, in float r1, in float r2 )
{
    float d1 = -p.y - h;
    float q = p.y - h;
    float si = 0.5*(r1-r2)/h;
    float d2 = max( sqrt( dot(p.xz,p.xz)*(1.0-si*si)) + q*si - r2, q );
    return length(max(vec2(d1,d2),0.0)) + min(max(d1,d2), 0.);
}

float sdPryamid4(vec3 p, vec3 h ) // h = { cos a, sin a, height }
{
    // Tetrahedron = Octahedron - Cube
    float box = sdBox( p - vec3(0,-2.0*h.z,0), vec3(2.0*h.z) );
 
    float d = 0.0;
    d = max( d, abs( dot(p, vec3( -h.x, h.y, 0 )) ));
    d = max( d, abs( dot(p, vec3(  h.x, h.y, 0 )) ));
    d = max( d, abs( dot(p, vec3(  0, h.y, h.x )) ));
    d = max( d, abs( dot(p, vec3(  0, h.y,-h.x )) ));
    float octa = d - h.z;
    return max(-box,octa); // Subtraction
 }

float length2( vec2 p )
{
	return sqrt( p.x*p.x + p.y*p.y );
}

float length6( vec2 p )
{
	p = p*p*p; p = p*p;
	return pow( p.x + p.y, 1.0/6.0 );
}

float length8( vec2 p )
{
	p = p*p; p = p*p; p = p*p;
	return pow( p.x + p.y, 1.0/8.0 );
}

// Функция для создания волнообразной поверхности
float waveSurface(vec2 p, float time)
{
    // Частота и амплитуда волны
    float frequency = 0.5;
    float amplitude = 0.2;

    // Используем синусоиду для создания волны
    return sin(p.x * frequency +  0.001 * time) * amplitude;
}

float sdTorus82( vec3 p, vec2 t )
{
    vec2 q = vec2(length2(p.xz)-t.x,p.y);
    return length8(q)-t.y;
}

float sdTorus88( vec3 p, vec2 t )
{
    vec2 q = vec2(length8(p.xz)-t.x,p.y);
    return length8(q)-t.y;
}

float sdCylinder6( vec3 p, vec2 h )
{
    return max( length6(p.xz)-h.x, abs(p.y)-h.y );
}

//------------------------------------------------------------------

float opS( float d1, float d2 )
{
    return max(-d2,d1);
}

vec2 opU( vec2 d1, vec2 d2 )
{
	return (d1.x<d2.x) ? d1 : d2;
}

vec3 opRep( vec3 p, vec3 c )
{
    return mod(p,c)-0.5*c;
}

vec3 opTwist( vec3 p )
{
    float  c = cos(10.0*p.y+10.0);
    float  s = sin(10.0*p.y+10.0);
    mat2   m = mat2(c,-s,s,c);
    return vec3(m*p.xz,p.y);
}

//------------------------------------------------------------------

float sdf_smin(float a, float b, float k)
{
    float res = exp(-k * a) + exp(-k * b);
    return -log(max(0.0001, res)) / k;
}


//------------------------------------------------------------------

vec2 map(in vec3 pos)
{   
    // Вращаем координаты точки вокруг Y-оси
    float yRotationSpeed = 0.5; // Скорость вращения вокруг Y-оси (измените по вашему желанию)
    float yAngle = u_time * yRotationSpeed; // Угол вращения вокруг Y-оси
    float cy = cos(yAngle);
    float sy = sin(yAngle);
    mat2 yRotationMatrix = mat2(cy, -sy, sy, cy);
    pos.xz = yRotationMatrix * pos.xz;

    // Вращаем координаты точки вокруг X-оси
    float xRotationSpeed = 0.5; // Скорость вращения вокруг X-оси (измените по вашему желанию)
    float xAngle = u_time * xRotationSpeed; // Угол вращения вокруг X-оси
    float cx = cos(xAngle);
    float sx = sin(xAngle);
    mat2 xRotationMatrix = mat2(cx, -sx, sx, cy);
    pos.yz = xRotationMatrix * pos.yz;
    float waveHeight = waveSurface(pos.xz, u_time);
    pos.y += waveHeight; 
	float waveHeight1 = waveSurface(pos.yz, u_time);
    pos.z += waveHeight; 
    float coef1 = mix(20.0, 18.0, 0.5 + 0.5 * sin(0.9653 * u_time));
	float coef2 = mix(24.0, 9.0, 0.5 + 0.5 * sin(0.7871 * u_time));
	float coef3 = mix(.0, 10.0, 0.5 + 0.5 * sin(0.854 * u_time));
    
    float coef11 = mix(8.0, 18.0, 0.5 + 0.5 * sin(1.9653 * u_time));
	float coef21 = mix(6.0, 9.0, 0.5 + 0.5 * sin(0.7871 * u_time));
	float coef31 = mix(4.0, 10.0, 0.5 + 0.5 * sin(0.354 * u_time));

	float sdf1 = 0.8 * sdSphere(pos - vec3(-0.0, 0.0, -0.0), 0.3) + 0.03 * sin(coef1 * pos.x) * sin(coef2 * pos.y) * sin(coef3 * pos.z);
    float sdf2 = 0.4 * sdSphere(pos - vec3(0.3, 0.59, 0.3), 0.1) + 0.03 * sin(coef11 * pos.x) * sin(coef21 * pos.y) * sin(coef31 * pos.z);

    float mergedSDF = sdf_smin(sdf1, sdf2, 16.0); // Измените параметр k по вашему желанию

    return vec2(mergedSDF, 600.0);
}

// Функция для расчета нормали
vec3 calcNormal( in vec3 pos )
{
    vec2 e = vec2(1.0,-1.0)*0.5773*0.0005;
    return normalize( e.xyy*map( pos + e.xyy ).x + 
                      e.yyx*map( pos + e.yyx ).x + 
                      e.yxy*map( pos + e.yxy ).x + 
                      e.xxx*map( pos + e.xxx ).x );
}

// Функция для броска луча
vec2 castRay( in vec3 ro, in vec3 rd )
{
    float tmin = 1.0;
    float tmax = 20.0;

    float t = tmin;
    float m = -1.0;
    for( int i=0; i<64; i++ )
    {
        float precis = 0.005*t;
        vec2 res = map( ro+rd*t );
        if( res.x<precis || t>tmax ) break;
        t += res.x;
        m = res.y;
    }

    if( t>tmax ) m = -1.0;
    return vec2( t, m );
}

float calcSoftshadow( in vec3 ro, in vec3 rd, in float mint, in float tmax )
{
	float res = 1.0;
    float t = mint;
    for( int i=0; i<16; i++ )
    {
		float h = map( ro + rd*t ).x;
        res = min( res, 8.0*h/t );
        t += clamp( h, 0.02, 0.10 );
        if( h<0.001 || t>tmax ) break;
    }
    return clamp( res, 0.0, 1.0 );
}

float fastSoftshadow(in vec3 ro, in vec3 rd, in float mint, in float tmax)
{
    float res = 1.0;
    float t = mint;
    for (int i = 0; i < 16; i++)
    {
        float h = map(ro + rd * t).x;
        res = min(res, 8.0 * h / t);
        t += max(h, 0.02);
        if (h < 0.001 || t > tmax)
            break;
    }
    return clamp(res, 0.0, 1.0);
}


float calcAO( in vec3 pos, in vec3 nor )
{
	float occ = 0.0;
    float sca = 1.0;
    for( int i=0; i<5; i++ )
    {
        float hr = 0.01 + 0.12*float(i)/4.0;
        vec3 aopos =  nor * hr + pos;
        float dd = map( aopos ).x;
        occ += -(dd-hr)*sca;
        sca *= 0.95;
    }
    return clamp( 1.0 - 3.0*occ, 0.0, 1.0 );    
}

// http://iquilezles.org/www/articles/checkerfiltering/checkerfiltering.htm
float checkersGradBox( in vec2 p )
{
    // filter kernel
    vec2 w = fwidth(p) + 0.001;
    // analytical integral (box filter)
    vec2 i = 2.0*(abs(fract((p-0.5*w)*0.5)-0.5)-abs(fract((p+0.5*w)*0.5)-0.5))/w;
    // xor pattern
    return 0.5 - 0.5*i.x*i.y;                  
}

vec3 pal( in float t, in vec3 a, in vec3 b, in vec3 c, in vec3 d ) {
  return a + b*cos( 6.28318*(c*t+d) );
}

vec3 spectrum(float n) {
  return pal( n, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(1.0,1.0,1.0),vec3(0.0,0.33,0.67) );
}

const float GAMMA = 2.2;

vec3 gamma(vec3 color, float g) {
  return pow(color, vec3(g));
}

vec3 linearToScreen(vec3 linearRGB) {
  return gamma(linearRGB, 1.0 / GAMMA);
}

vec3 render( in vec3 ro, in vec3 rd )
{ 
    vec3 col = vec3(0.7, 0.9, 1.0) + rd.y * 0.8;
    vec2 res = castRay(ro, rd);
    float t = res.x;
    float m = res.y;
    if (m > -0.5)
    {
        vec3 pos = ro + t * rd;
        vec3 nor = calcNormal(pos);
        vec3 ref = reflect(rd, nor);
        
        // material        
        col =  nor *0.45 + 0.15 * sin(vec3(0.05, 0.08, 0.10) * (m - 1.0));
        if (m < 1.5)
        {
            float f = checkersGradBox(5.0 * pos.xz);
            col = 0.3 + f * vec3(0.1);
        }
        
        

        // lighitng
        float occ = calcAO(pos, nor); // Рассчет окклюзии (теней)
		vec3 lig = normalize(vec3(-0.4, 0.7, -0.6)); // Направление источника света
		vec3 hal = normalize(lig - rd); // Вектор наполовину между направлением наблюдения и направлением источника света
		float amb = clamp(0.5 + 0.5 * nor.y, 0.0, 1.0); // Рассчет фоновой (амбиентной) составляющей освещения
		float dif = clamp(dot(nor, lig), 0.0, 1.0); // Рассчет диффузной составляющей освещения
		float bac = clamp(dot(nor, normalize(vec3(-lig.x, 0.0, -lig.z))), 0.0, 1.0) * clamp(1.0 - pos.y, 0.0, 1.0); // Рассчет зеркального блика
		float dom = smoothstep(-0.1, 0.1, ref.y); // Рассчет доминантной составляющей освещения
		float fre = pow(clamp(1.0 + dot(nor, rd), 0.0, 1.0), 2.0); // Рассчет зеркального отражения (Френеля)

        
        
        dif *= fastSoftshadow(pos, lig, 0.02, 2.5); // Рассчет мягких теней для диффузной составляющей
		
        dom *= fastSoftshadow(pos, ref, 0.02, 2.5); // Рассчет мягких теней для доминантной составляющей


        float spe = pow(clamp(dot(nor, hal), 0.0, 1.0), 16.0) *
            	dif *
            	(0.04 + 0.96 * pow(clamp(1.0 + dot(hal, rd), 0.0, 1.0), 5.0)); // Рассчет спекулярных бликов


        vec3 lin = vec3(0.0);
        lin += 1.30 * dif * vec3(1.00, 0.80, 0.55);
        lin += 1.30 * dif * vec3(1.00, 0.80, 0.55); // Диффузная составляющая света
		lin += 0.40 * amb * vec3(0.40, 0.60, 1.00) * occ; // Амбиентная составляющая света
		lin += 0.50 * dom * vec3(0.40, 0.60, 1.00) * occ; // Доминантная составляющая света
		lin += 1.50 * bac * vec3(0.25, 0.25, 0.25) * occ; // Зеркальный блик
		lin += 1.25 * fre * vec3(1.00, 1.00, 1.00) * occ; // Зеркальное отражение (Френель)

        // Добавляем эффект перламутровости
        float pearlIntensity = 0.0; // Интенсивность перламутровости
        vec3 pearlColor = vec3(1.000,0.066,0.807); // Цвет перламутра
        vec3 pearlShift = vec3(0.875,0.052,0.061); // Сдвиг для создания мерцающего эффекта
        float pearlEffect = (sin(pos.x * pearlShift.x) + sin(pos.y * pearlShift.y) + sin(pos.z * pearlShift.z)) * pearlIntensity;
        col = mix(col, pearlColor, pearlEffect);

        col = col * lin;
        col += 10.00 * spe * vec3(1.00, 0.90, 0.70);

        col = mix(col, vec3(0.8, 0.9, 1.0), 1.0 - exp(-0.0002 * t * t * t));
    }

    return vec3(clamp(col, 0.0, 1.0));
}

mat3 setCamera( in vec3 ro, in vec3 ta, float cr )
{
    // Нормализуем вектор направления взгляда (передней части камеры)
    vec3 cw = normalize(ta - ro);

    // Создаем вектор, указывающий вверх (по y) камеры и поворачиваем его на угол cr
    vec3 cp = vec3(sin(cr), cos(cr), 0.0);

    // Находим вектор, перпендикулярный плоскости, образуемой cw и cp
    vec3 cu = normalize(cross(cw, cp));

    // Находим вектор, перпендикулярный плоскости, образуемой cu и cw
    vec3 cv = normalize(cross(cu, cw));

    // Возвращаем ортонормальную матрицу камеры, в которой
    // cu, cv и cw являются базисными векторами направлений
    return mat3(cu, cv, cw);
}


void main()
{
    vec2 mo = u_mouse.xy / u_resolution.xy;
    float time = 15.0 + u_time;
    vec3 tot = vec3(0.0);

    vec2 p = (-u_resolution.xy + 2.0 * gl_FragCoord.xy) / u_resolution.y;
    vec3 ro = vec3(0.5 + 3.5 * cos(0.0 * time + 6.0 * mo.x), 1.0 + 3.0 * mo.y, 0.5 + 3.5 * sin(0.0 * time + 6.0 * mo.x));
	vec3 ta = vec3(0.000, 0.700, 0.000);
    mat3 ca = setCamera(ro, ta, 0.0);
    vec3 rd = ca * normalize(vec3(p.xy, 2.0));

    vec3 col = render(ro, rd);
    col = pow(col, vec3(0.4545));
    tot += col;

    gl_FragColor = vec4(tot, 1.0);
}