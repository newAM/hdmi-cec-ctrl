use log::{LevelFilter, Metadata, Record};

pub struct Logger {
    level_filter: LevelFilter,
}

impl Logger {
    pub const fn new(level_filter: LevelFilter) -> Self {
        Self { level_filter }
    }
}

impl log::Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        self.level_filter.ge(&metadata.level())
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            rtt_target::rprintln!("[{}] [{}] {}", super::now(), record.level(), record.args());
        }
    }

    fn flush(&self) {}
}
