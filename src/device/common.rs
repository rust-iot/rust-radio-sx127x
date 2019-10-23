/// Payload length configuration
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum PayloadLength {
    /// Constant length payloads use implicit headers
    Constant(u16),
    /// Variable length payloads must be contain explicit headers
    Variable,
}
