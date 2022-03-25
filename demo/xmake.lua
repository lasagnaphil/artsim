local demos = {
    "demo_spheres_pbr",
    "demo_basic",
    "demo_pbd",
    "demo_human",
    "demo_soft_body"
}

for i, demo_name in ipairs(demos) do
    target("gengine_" .. demo_name)
    set_kind("binary")
    add_files(demo_name .. ".cpp")
    add_deps("gengine")
end