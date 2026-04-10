use crate::model::Model;

pub fn generate_c_header_summary(model: &Model) -> String {
    format!(
        "/* pinocchio_wasm generated summary */\n/* nlinks={} nv={} */\n",
        model.nlinks(),
        model.nv()
    )
}

pub fn generate_js_loader(module_name: &str) -> String {
    format!(
        "export async function load{0}(path) {{\n  const bytes = await fetch(path).then(r => r.arrayBuffer());\n  return WebAssembly.instantiate(bytes, {{}});\n}}\n",
        module_name
    )
}
