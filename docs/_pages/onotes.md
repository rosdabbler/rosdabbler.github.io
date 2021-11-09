---
permalink: /onotes/
title: "Notes by category"
collection: onotes
---
These are somewhat raw notes taken while investigating various issues. They may be incomplete or hard to follow without the original context.
{% for cat in site.category-list %}
### {{ cat }}
<ul>
  {% for page in site.notes %}
    {% for pc in page.categories %}
    {% if pc == cat %}
        <li><a href="{{ page.url }}">{{ page.title }}</a>
            {% include page__date.html %}
        </li>
    {% endif %}   <!-- cat-match-p -->
    {% endfor %}  <!-- page-category -->
  {% endfor %}  <!-- page -->
</ul>
{% endfor %}  <!-- cat -->