function saveFig(fig, path)
    fig.PaperPositionMode = 'auto';
    fig_pos = fig.PaperPosition;
    fig.PaperSize = [fig_pos(3) fig_pos(4)+0.1];
    print(fig, path, '-dpdf')
end

