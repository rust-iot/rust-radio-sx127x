/// Payload length configuration
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum PayloadLength {
    /// Constant length payloads use implicit headers
    Constant(u16),
    /// Variable length payloads must be contain explicit headers
    Variable,
}
