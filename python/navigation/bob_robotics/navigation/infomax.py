import bob_robotics.navigation as bobnav
InfoMax = bobnav.InfoMax


@bobnav.cache_result
def get_trained_network(train_images, seed,
                        learning_rate=InfoMax.DEFAULT_LEARNING_RATE):
    assert seed is not None

    return InfoMax(
        train_images=train_images, learning_rate=learning_rate, seed=seed)


@bobnav.cache_result
def get_infomax_headings(ann, images):
    df = ann.ridf(images)
    return df.estimated_heading
