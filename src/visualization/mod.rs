use crate::model::Model;

pub fn ascii_tree(model: &Model) -> String {
    fn dfs(model: &Model, node: usize, depth: usize, out: &mut String) {
        for _ in 0..depth {
            out.push_str("  ");
        }
        out.push_str(&model.links[node].name);
        out.push('\n');
        for &c in model.children_of(node) {
            dfs(model, c, depth + 1, out);
        }
    }

    let mut s = String::new();
    dfs(model, 0, 0, &mut s);
    s
}
