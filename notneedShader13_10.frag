precision mediump float;
uniform vec2 u_resolution;           // viewport resolution (in pixels)
uniform float u_time;                 // shader playback time (in seconds)
uniform vec2 u_mouse;                 // shader playback time (in seconds)
// Определение константных параметров для эффекта
#define resolution 3.       // Разрешение сетки
#define waveFrequency 0.5    // Частота волн
#define waveSize 1.75        // Размер волн
#define simulationSpeed 0.1  // Скорость симуляции
#define baseColorIntensity 0.0 // Интенсивность базового цвета



vec3 baseColor = vec3(0.0); // Определение базового цвета

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
// Функция для вычисления маски волны в данной точке
float circle(vec2 uv, float radius, float blur, vec2 offset){
    float dist = distance(uv, offset); 
    return smoothstep(radius, radius - blur, dist); 
}

void main()
{
    vec2 uv = gl_FragCoord.xy / u_resolution.xy; // Нормализация координат пикселя
    uv -= .5;
    uv.x *= (1.0 * u_resolution.x ) / (1.0 * u_resolution.y);

    float mask = 0.;
    const float multiplicativeInverseResolution = 1. / resolution;
    float t = u_time * simulationSpeed;
    float z = 3.;
    vec3 floatMask = vec3(0.);
    float angle = radians(45.0); // Угол поворота (в радианах)
    mat2 rotationMatrix = mat2(cos(angle), -sin(angle), sin(angle), cos(angle));
    // Цикл для создания волны
    for(float y = -3.5; y < 3.5; y += multiplicativeInverseResolution){
        vec2 uvOffset = uv * z;
        //for (float x = -.5; x < .5; x += multiplicativeInverseResolution){    
        //    vec2 offset = vec2(x + sin((t + y) * waveFrequency) * waveSize, y + cos((t + x) * waveFrequency) * (waveSize * 1.5)
        //     + computeValue(iMouse.xy , iResolution.xy) * (sin((t + y + 2.4) * 5.5 * waveFrequency) * 0.5 * waveSize, cos((t + x + 2.4) * waveFrequency) * (waveSize * 1.5)) + ( x + sin((t + y + 0.4) * 10.5 * waveFrequency) * 10.5 * waveSize, y + cos((t + x + 0.4) * waveFrequency) * (waveSize * 1.5)));                	
        //    mask += circle(uvOffset, (computeValue(iMouse.xy , iResolution.xy) + 1.) * .015, 0.005, offset * vec2(2., 1.0) + vec2(0, 0.1));
        //}
        for (float x = -3.5; x < 3.5; x += multiplicativeInverseResolution){    
        vec2 offset = vec2(
            0.63 * cos( x + sin((t + y ) * waveFrequency) * waveSize) * cos(y + cos((t + x + 0.2) * waveFrequency) * (waveSize * 1.5)),
            0.9 * sin( x - sin((t + y ) * waveFrequency) * waveSize) * cos(y + cos((t + x) * waveFrequency) * (waveSize * 1.5))
            );
            


            mask += circle(uvOffset, (computeValue(u_mouse.xy , u_resolution.xy) + 1.) * .025, 0.005, offset * vec2(5., 1.0) + vec2(0.9, .1));
        }
        z += multiplicativeInverseResolution * 1.;
    }

    baseColor *= baseColorIntensity; // Изменение интенсивности базового цвета
    vec3 color = 0.5 + 0.5 * cos(u_time + uv.xyx + vec3(10. , 20. , 0. )); // Вычисление цвета для фона

    gl_FragColor = vec4(baseColor + color  * mask, 1.0); // Установка итогового цвета пикселя
}
