#[macro_export]
/// THIS SHOULD NOT BE PUBLIC
macro_rules! timed {
    ($name:literal, $block:block) => {{
        #[cfg(feature = "stats")]
        {
            let start = std::time::Instant::now();
            let result = $block;
            log::debug!("{} in {:?}", $name, start.elapsed());
            result
        }

        #[cfg(not(feature = "stats"))]
        {
            $block
        }
    }};
}
