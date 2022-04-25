import bob_robotics.navigation as bobnav
InfoMax = bobnav.InfoMax


@bobnav.cache_result
def get_trained_network(training_images, seed,
                        learning_rate=InfoMax.DEFAULT_LEARNING_RATE):
    assert seed is not None

    infomax = InfoMax(training_images[0].shape[::-1], learning_rate=learning_rate, seed=seed)
    infomax.train(training_images)
    return infomax


@bobnav.cache_result
def get_infomax_headings(ann, images):
    df = ann.ridf(images)
    return df.estimated_heading
