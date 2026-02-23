#[derive(Clone, Debug, Hash)]
pub struct TfGraphNode {
    pub child: String,
    pub parent: String,
}

impl PartialEq for TfGraphNode {
    fn eq(&self, other: &Self) -> bool {
        self.child == other.child && self.parent == other.parent
    }
}

impl Eq for TfGraphNode {}
