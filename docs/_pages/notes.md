---
permalink: /notes/
title: "Notes by tag"
collection: notes
layout: single
---
These are somewhat raw notes taken while investigating various issues. They may be incomplete or hard to follow without the original context.

<!-- generate tags in notes, and list of notes for each tag -->

{% assign tags =  site.notes | map: 'tags' | join: ','  | split: ',' | uniq %}
{% assign tags_notes = "" | split: ',' %}
{% for tag in tags %}
  {% assign tag_notes = "" | split: ',' %}
  {% for note in site.notes %}
    {% if note.tags contains tag %}
      {% assign tag_notes = tag_notes | push: note %}
    {% endif %}
  {% endfor %} <!-- note -->
  {% assign tags_notes = tags_notes | push: tag_notes %}
{% endfor %} <!-- tags -->

<h2>Tags and counts</h2>
<ul class="taxonomy__index">
{% for tag in tags %}
  <li>
    <a href="#{{ tag | slugify | downcase }}">
      <strong>{{ tag }}</strong>
      <span class="taxonomy__count">{{ tags_notes[forloop.index0] | size }}</span>
    </a>
  </li>
{% endfor %}
</ul>

<h2>Titles by Tag</h2>
<section class="taxonomy__section">
{% for tag in tags %}
    {% assign tag_notes = tags_notes[forloop.index0] %}
    <section id="{{ tag | slugify | downcase }}">
    <h3>{{ tag }}</h3>
    <div class="entries-list">
      <div>
        {% for note in tag_notes %}
        <li><a href="{{ note.url }}">{{ note.title }}</a> ( {{ note.date | date_to_long_string }} )
        </li>
        {% endfor %}
      </div>
    </div>
    <a href="#page-title" class="back-to-top">{{ site.data.ui-text[site.locale].back_to_top | default: 'Back to Top' }} &uarr;</a>
    </section>
{% endfor %}
</section>

