#version 330 core

struct Material {
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    float shininess;
    sampler2D texDiffuse;
    sampler2D texSpecular;
    bool useTexDiffuse;
    bool useTexSpecular;
};

struct DirLight {
    bool enabled;

    vec3 direction;

    vec4 ambient;
    vec4 diffuse;
    vec4 specular;

    float intensity;
};

struct PointLight {
    bool enabled;

    vec3 position;

    float constant;
    float linear;
    float quadratic;

    vec4 ambient;
    vec4 diffuse;
    vec4 specular;

    float intensity;
};

struct SpotLight {
    bool enabled;

    vec3 position;
    vec3 direction;
    float cutOff;
    float outerCutOff;

    vec4 ambient;
    vec4 diffuse;
    vec4 specular;

    float constant;
    float linear;
    float quadratic;

    float intensity;
};

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec4 FragPos_DirLightSpace;
} fs_in;

out vec4 fragColor;

uniform vec3 viewPos;

uniform Material material;

uniform DirLight dirLight;

#define NR_POINT_LIGHTS 16
uniform PointLight pointLights[NR_POINT_LIGHTS];

#define NR_SPOT_LIGHTS 8
uniform SpotLight spotLights[NR_SPOT_LIGHTS];

uniform int numPointLights;
uniform int numSpotLights;

uniform sampler2D shadowMap;

float shadowCalculation(vec4 fragPosLightSpace) {
    float bias = 0.0005;
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;
    float currentDepth = projCoords.z;
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
            shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;
        }
    }
    shadow /= 9.0;
    return shadow;
}

vec4 calcDirLight(DirLight light, vec3 normal, vec3 viewDir, vec4 matDiffuse, vec4 matSpecular) {
    vec3 lightDir = normalize(-light.direction);

    float diff = max(dot(normal, lightDir), 0.0);

    // vec3 reflectDir = reflect(-lightDir, normal);
    // float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(viewDir, halfwayDir), 0.0), material.shininess);

    vec4 ambient = light.ambient * matDiffuse;
    vec4 diffuse = light.intensity * light.diffuse * diff * matDiffuse;
    vec4 specular = light.intensity * light.specular * spec * matSpecular;
    float shadow = shadowCalculation(fs_in.FragPos_DirLightSpace);

    return ambient + (1.0 - shadow) * (diffuse + specular);
}

vec4 calcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir, vec4 matDiffuse, vec4 matSpecular) {
    vec3 lightDir = normalize(light.position - fragPos);

    float diff = max(dot(normal, lightDir), 0.0);

    // vec3 reflectDir = reflect(-lightDir, normal);
    // float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(viewDir, halfwayDir), 0.0), material.shininess);

    float distance = length(light.position - fragPos);
    float attenuation = 1.0f / (light.constant + light.linear * distance + light.quadratic * (distance * distance));

    vec4 ambient = light.ambient * matDiffuse;
    vec4 diffuse = light.intensity * light.diffuse * diff * matDiffuse;
    vec4 specular = light.intensity * light.specular * spec * matSpecular;
    float shadow = shadowCalculation(fs_in.FragPos_DirLightSpace);

    return attenuation * (ambient + (1.0 - shadow) * (diffuse + specular));
}

vec4 calcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir, vec4 matDiffuse, vec4 matSpecular) {
    vec3 lightDir = normalize(light.position - fragPos);

    float theta = dot(lightDir, normalize(-light.direction));
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);

    float diff = max(dot(normal, lightDir), 0.0);

    // vec3 reflectDir = reflect(-lightDir, normal);
    // float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(viewDir, halfwayDir), 0.0), material.shininess);

    float distance = length(light.position - fragPos);
    float attenuation = 1.0f / (light.constant + light.linear * distance + light.quadratic * (distance * distance));

    vec4 ambient = light.ambient * matDiffuse;
    vec4 diffuse = light.intensity * light.diffuse * diff * matDiffuse;
    vec4 specular = light.intensity * light.specular * spec * matSpecular;
    float shadow = shadowCalculation(fs_in.FragPos_DirLightSpace);

    return attenuation * (ambient + (1.0 - shadow) * intensity * (diffuse + specular));
}

void main() {
    vec3 norm = normalize(fs_in.Normal);
    vec3 viewDir = normalize(viewPos - fs_in.FragPos);

    vec4 matDiffuse, matSpecular;
    if (material.useTexDiffuse)
        matDiffuse = texture(material.texDiffuse, fs_in.TexCoords);
    else
        matDiffuse = material.diffuse;

    if (material.useTexSpecular)
        matSpecular = texture(material.texSpecular, fs_in.TexCoords);
    else
        matSpecular = material.specular;

    vec4 result = vec4(0.0);
    if (dirLight.enabled) result = calcDirLight(dirLight, norm, viewDir, matDiffuse, matSpecular);

    for (int i = 0; i < numPointLights; ++i) {
        if (pointLights[i].enabled) {
            result += calcPointLight(pointLights[i], norm, fs_in.FragPos, viewDir, matDiffuse, matSpecular);
        }
    }

    for (int i = 0; i < numSpotLights; ++i) {
        if (spotLights[i].enabled) {
            result += calcSpotLight(spotLights[i], norm, fs_in.FragPos, viewDir, matDiffuse, matSpecular);
        }
    }

    fragColor = result;
}
