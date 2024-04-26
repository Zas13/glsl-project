precision mediump float;


uniform vec2 u_resolution;           // Разрешение экрана (в пикселях)

uniform float u_time;                // Время работы шейдера (в секундах)

uniform vec2 u_mouse; 


const float MAX_TRACE_DISTANCE = 5.5;         // Максимальное расстояние для трассировки лучей

const float INTERSECTION_PRECISION = 0.001;    // Точность определения пересечения

const int NUM_OF_TRACE_STEPS = 170;            // Количество шагов трассировки

const float SCALE_DIST = 0.17;                 // Масштабирование расстояния

const float K = 0.3;                         // К в функции сглаженного объединения

const float ofX = 0.0;                    // Смещение по Х

const float ofY = 0.0;					// Смещение по Y

const float PI = 3.14159265359;


// Палитра цветов "спектр"

vec3 pal( in float t, in vec3 a, in vec3 b, in vec3 c, in vec3 d ) {

    return a + b*cos( 6.28318*(c*t+d) );

}


// Генерация цвета на основе спектра

vec3 spectrum(float n) {

    return pal( n, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(1.0,1.0,1.0),vec3(0.2667, 0.6706, 0.0) );

}


vec3 spectrum2(float n) {

    return pal( n, vec3(0.6745, 0.6745, 0.6745),vec3(0.5, 0.5, 0.5),vec3(0.5, 0.5, 0.5),vec3(0.5, 0.5, 0.5) );

}




float computeValue(vec2 u_mouse, vec2 u_resolution) {

    vec2 mousePos = u_mouse / u_resolution;

    float distanceToCenter = length(mousePos - vec2(0.5));

    
    if (distanceToCenter <= 0.2) {

        return 1.0;

    } else if (distanceToCenter >= 0.2 && distanceToCenter <= 0.3) {

        return 3.- 10.*distanceToCenter;

    } else {

        return 0.0;

    }

}




// Теперь 'value' будет равно 1 в центре экрана и уменьшаться по мере приближения к краю.



// Вычисление матрицы обзора камеры

mat3 calcLookAtMatrix( in vec3 ro, in vec3 ta, in float roll )

{

    vec3 ww = normalize( ta - ro );

    vec3 uu = normalize( cross(ww,vec3(sin(roll),cos(roll),0.0) ) );

    vec3 vv = normalize( cross(uu,ww));

    return mat3( uu, vv, ww );

}


// Функция для определения расстояния до сферы

float sdSphere( vec3 p, float s )

{

  return length(p)-s;

}


// Функция для объединения объектов

float sdSub( float d0, float d1 ) {

    return max( d0, -d1 );

}



float opOnion( in float sdf, in float thickness )

{

    return abs(sdf)-thickness;

}


// Функция для объединения объектов сглаживающим сплайном

float sdUnion_s( float a, float b, float k ) {

    float h = clamp( 0.5+0.5*(b-a)/k, 0.0, 1.0 );

    return mix( b, a, h ) - k*h*(1.0-h);

}


// Преобразование из декартовых координат в полярные

vec3 carToPol(vec3 p) {

    float r = length(p);

    float the = acos(p.z/r);

    float phi = atan(p.y,p.x);

    return vec3(r,the,phi);

}


// Функция для маппинга объектов в сцене

vec2 map( vec3 pos ){


    // Коэффициенты для создания волн в объектах

    float coef1 = mix(8.0, 9.0, 0.5 + 0.5 * sin(0.9653 * u_time +8.5));

    float coef2 = mix(12.0, 9.0, 0.5 + 0.5 * sin(0.7871 * u_time+2.8));

    float coef3 = mix(1.0, 5.0, 0.5 + 0.5 * sin(0.854 * u_time+11.1));

    float coef4 = mix(1.0, 9.0, 0.5 + 0.5 * cos(0.7871 * u_time+2.8));

    // Создание объекта 4 (сфера с волнами)

    vec3 p4 = vec3(0.0, 0.0, 0.0);

    float d4 = 1.4 * sdSphere(pos - vec3(-0.0, 0.0, -0.0), 0.8) + 0.09 * cos(0.3 *coef1 * pos.x) * cos(0.6 *coef2 * pos.y) * cos(coef3 * pos.z);

    float d6 = 1.4 * sdSphere(pos - vec3(-0.0, 0.0, -0.0), computeValue(u_mouse, u_resolution)+0.3*0.1) + 0.09 * sin(0.3 *coef1 * pos.x) * sin(0.6 *coef2 * pos.y) * sin(coef3 * pos.z);

    float d0 = sdUnion_s(d4, d6, K);                         // К в функции сглаженного объединения); // Просто используем d4, так как нет p2 и p3

    //computeValue(u_mouse, u_resolution)

    // Преобразование в полярные координаты

    vec3 pol = carToPol(pos);

    // Создание объекта 1 (сфера с волнами)

    float d1 = 4.9 * sdSphere(pos - vec3(-0.0, 0.0, -0.0), 1.-computeValue(u_mouse, u_resolution)) + 0.09 * sin(coef1 * pos.x) * sin(coef2 * pos.y) * sin(coef3 * pos.z);

    float wave = 0.15*sin((coef1+coef4/coef1-coef4)*(pol.y))*sin(3.0*pol.z)*sin(22.0*pol.x);

    float wave1 = 0.55*sin((coef1)*(pol.y))*sin(3.0*pol.z)*sin(1.0*pol.x);

    d0 = opOnion(d0+computeValue(u_mouse, u_resolution)/1.*wave1, 0.001);

    d1 = opOnion(d1+wave, 0.001);

    
    // Определение id для каждого объекта

    float id0 = 1.0; // Устанавливаем id для объекта d0

    float id1 = 2.0; // Устанавливаем id для объекта d1


    vec2 res;

    // Выбор результата в зависимости от объекта

    if (d0 < d1) {

        res = vec2(d0, id0);

    } else {

        res = vec2(d1, id1);

    }


    return res;

}


// Функция для расчета цвета объектов

vec3 selfColor(vec3 pos, float id) {

    vec3 pol = carToPol(pos);

    vec3 color = vec3(0.0); // Инициализируем цвет нулевым значением


    // В зависимости от id вычисляем цвет

    if (id == 1.0) {

        // Цвет для объекта с id = 1

        color = spectrum(0.1*computeValue(u_mouse, u_resolution)+1.*pol.z/PI/2.0+0.0*pol.y/PI);

    } else if (id == 2.0) {

        // Цвет для объекта с id = 2

        color = spectrum2(1.*computeValue(u_mouse, u_resolution)*pol.z/PI/2.0+0.*pol.y/PI);

    }


    return color;

}


// Функция для расчета пересечения лучей с объектами

vec2 calcIntersection( in vec3 ro, in vec3 rd , inout vec3 col){


    float h =  MAX_TRACE_DISTANCE;

    float t = 0.0;

	float res = -1.0;

    float id = -1.;

    
    for( int i=0; i< NUM_OF_TRACE_STEPS ; i++ ){

        
        if(t > MAX_TRACE_DISTANCE ) break;

	   	vec2 m = map( ro+rd*t );

        h = m.x;

        
        vec3 pos = ro + rd * t;

        col += selfColor(pos, id) * exp(-5.0*h)*0.007 ;

        
        t += max(abs(h)*SCALE_DIST, INTERSECTION_PRECISION);

        id = m.y;

    }


    if( t < MAX_TRACE_DISTANCE ) res = t;

    if( t > MAX_TRACE_DISTANCE ) id =-1.0;

    
    return vec2( res , id );

}


// Функция для вращения камеры по осям X и Y

mat3 rotationXY( vec2 angle ) {

	vec2 c = cos( angle );

	vec2 s = sin( angle );

	
	return mat3(

		c.y      ,  0.0, -s.y,

		s.y * s.x,  c.x,  c.y * s.x,

		s.y * c.x, -s.x,  c.y * c.x

	);

}


void main()

{

    vec2 offset = vec2(ofX, ofY);
    
    vec2 p_offset = (- 1.0 * u_resolution.xy + 2.0*gl_FragCoord.xy)/u_resolution.y;
    
    vec2 p = p_offset + offset;
    
    vec3 ro = vec3( 3.0*cos(0.2*u_time), 0.0, 3.0*sin(0.2*u_time));

    vec3 ta = vec3( 0. , 0. , 0. );

    
    // Матрица обзора камеры

    mat3 camMat = calcLookAtMatrix( ro, ta, sin(0.3*u_time) );  // 0.0 - это угол наклона камеры

    
	// Создание вида

	vec3 rd = normalize( camMat * vec3(p.xy,1.0) ); // 1.0 - это фокусное расстояние Расстояние до объекта от камеры 

    

    

    // Вращение камеры

	//mat3 rot = rotationXY( ( u_mouse.xy - u_resolution.xy * 0.5 ).yx * vec2( 0.01, -0.01 ) );

	//rd = rot * rd;

	//ro = rot * ro;


    vec3 accCol = vec3(0.0, 0.0, 0.0);

    
    vec2 res = calcIntersection( ro , rd, accCol);


	vec3 color = accCol;

	gl_FragColor = vec4(color,1.0);

}
