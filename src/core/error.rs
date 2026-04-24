use core::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PinocchioError {
    InvalidModel(String),
    DimensionMismatch { expected: usize, got: usize },
    SingularMatrix,
    IndexOutOfBounds { index: usize, len: usize },
}

impl PinocchioError {
    pub fn invalid_model(msg: impl Into<String>) -> Self {
        PinocchioError::InvalidModel(msg.into())
    }
}

impl fmt::Display for PinocchioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PinocchioError::InvalidModel(msg) => write!(f, "invalid model: {msg}"),
            PinocchioError::DimensionMismatch { expected, got } => {
                write!(f, "dimension mismatch: expected {expected}, got {got}")
            }
            PinocchioError::SingularMatrix => write!(f, "matrix is singular"),
            PinocchioError::IndexOutOfBounds { index, len } => {
                write!(f, "index {index} is out of bounds (len={len})")
            }
        }
    }
}

impl std::error::Error for PinocchioError {}

pub type Result<T> = std::result::Result<T, PinocchioError>;
