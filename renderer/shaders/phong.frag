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

out vec4 fragColor;

in vec3 fragPos;
in vec3 normal;
in vec2 texCoord;

uniform vec3 viewPos;

uniform Material material;

#define NR_POINT_LIGHTS 16
#define NR_SPOT_LIGHTS 8

uniform DirLight dirLight;
uniform PointLight pointLights[NR_POINT_LIGHTS];
uniform SpotLight spotLights[NR_SPOT_LIGHTS];

uniform int numPointLights;
uniform int numSpotLights;

vec4 calcDirLight(DirLight light, vec3 normal, vec3 viewDir, vec4 matDiffuse, vec4 matSpecular) {
    vec3 lightDir = normalize(-light.direction);

    float diff = max(dot(normal, lightDir), 0.0);

    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(viewDir, halfwayDir), 0.0), material.shininess);

    vec4 ambient = light.ambient * matDiffuse;
    vec4 diffuse = light.intensity * light.diffuse * diff * matDiffuse;
    vec4 specular = light.intensity * light.specular * spec * matSpecular;

    return (ambient + diffuse + specular);
}

vec4 calcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir, vec4 matDiffuse, vec4 matSpecular) {
    vec3 lightDir = normalize(light.position - fragPos);

    float diff = max(dot(normal, lightDir), 0.0);

    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(viewDir, halfwayDir), 0.0), material.shininess);

    float distance = length(light.position - fragPos);
    float attenuation = 1.0f / (light.constant + light.linear * distance + light.quadratic * (distance * distance));

    vec4 ambient = light.ambient * matDiffuse;
    vec4 diffuse = light.intensity * light.diffuse * diff * matDiffuse;
    vec4 specular = light.intensity * light.specular * spec * matSpecular;

    return attenuation * (ambient + diffuse + specular);
}

vec4 calcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir, vec4 matDiffuse, vec4 matSpecular) {
    vec3 lightDir = normalize(light.position - fragPos);

    float theta = dot(lightDir, normalize(-light.direction));
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);

    float diff = max(dot(normal, lightDir), 0.0);

    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(viewDir, halfwayDir), 0.0), material.shininess);

    float distance = length(light.position - fragPos);
    float attenuation = 1.0f / (light.constant + light.linear * distance + light.quadratic * (distance * distance));

    vec4 ambient = light.ambient * matDiffuse;
    vec4 diffuse = light.intensity * light.diffuse * diff * matDiffuse;
    vec4 specular = light.intensity * light.specular * spec * matSpecular;

    return attenuation * (ambient + intensity * (diffuse + specular));
}

void main() {
    vec3 norm = normalize(normal);
    vec3 viewDir = normalize(viewPos - fragPos);
    vec4 matDiffuse = material.useTexDiffuse? texture(material.texDiffuse, texCoord) : material.diffuse;
    vec4 matSpecular = material.useTexSpecular? texture(material.texSpecular, texCoord) : material.specular;

    vec4 result = vec4(0.0);
    if (dirLight.enabled) result = calcDirLight(dirLight, norm, viewDir, matDiffuse, matSpecular);

    for (int i = 0; i < numPointLights; ++i) {
        if (pointLights[i].enabled) {
            result += calcPointLight(pointLights[i], norm, fragPos, viewDir, matDiffuse, matSpecular);
        }
    }

    for (int i = 0; i < numSpotLights; ++i) {
        if (spotLights[i].enabled) {
            result += calcSpotLight(spotLights[i], norm, fragPos, viewDir, matDiffuse, matSpecular);
        }
    }

    fragColor = result;
}
